#include "LaneDetector.h"
#include <errno.h>
#include <poll.h>
#include <cmath>
#include "sensor_msgs/Image.h"
#include "std_msgs/ColorRGBA.h"
#include "cv_bridge/cv_bridge.h"

using namespace std;

cv::Mat* current_img = NULL;
pthread_rwlock_t imgproc_semaphore = PTHREAD_RWLOCK_INITIALIZER;

void img_listener(const sensor_msgs::Image& img)
{
    // printf("new image\n");
    pthread_rwlock_wrlock(&imgproc_semaphore);
    // copy the image onto the heap meaning it can exist without the global binding
    current_img = new cv::Mat(cv_bridge::toCvCopy(img, string("bgr8"))->image);
    pthread_rwlock_unlock(&imgproc_semaphore);
}

void* lane_detection_loop(void* detector_ptr)
{
    LaneDetector* detector = (LaneDetector*)detector_ptr;

    pthread_rwlock_rdlock(&detector->exit_semaphore);
    bool running = detector->running;
    pthread_rwlock_unlock(&detector->exit_semaphore);

    while (running)
    {
        struct timeval now;
        gettimeofday(&now, NULL);
        unsigned long start_time = now.tv_sec * 1000 + now.tv_usec / 1000;

        detector->detect_lane();

        gettimeofday(&now, NULL);
        unsigned long end_time = now.tv_sec * 1000 + now.tv_usec / 1000;

        usleep(1000000 - ((end_time - start_time) * 1000));

        pthread_rwlock_rdlock(&detector->exit_semaphore);
        running = detector->running;
        pthread_rwlock_unlock(&detector->exit_semaphore);
    }

    return NULL;
}

LaneDetector::LaneDetector() : running(true), median_blur_radius(25), rosnode(ros::NodeHandle()),
        canny_grad_thresh(80), canny_cont_thresh(30), hough_radius_inc(10),
        hough_theta_inc(4.0 * CV_PI / 180.0), hough_min_votes(300)
{
    laneimg_listener = rosnode.subscribe("camera/rgb/image_rect_color", 2, img_listener);
    pose_publisher = rosnode.advertise<std_msgs::ColorRGBA>("lane_pose", 2);

    if (pthread_rwlock_init(&exit_semaphore, NULL) == -1)
    {
        throw runtime_error(string("pthread_rwlock_init: failed to initialize LaneDetector.exit_semaphore: ") + to_string(errno));
    }

    if (pthread_create(&lane_detection_thread, NULL, &lane_detection_loop, (void*)this) == -1)
    {
        throw runtime_error(string("pthread_create(): failed to start background processing thread: ") + to_string(errno));
    }
}

LaneDetector::~LaneDetector()
{
    pthread_rwlock_wrlock(&exit_semaphore);
    running = false;
    pthread_rwlock_unlock(&exit_semaphore);

    pthread_join(lane_detection_thread, NULL);
}

double detection_confidence(const vector<cv::Vec2d>& lane_lines_left,
        const vector<cv::Vec2d>& lane_lines_right)
{
    const int RADIUS_L = 0;
    const int THETA_L = 1;
    const int RADIUS_R = 2;
    const int THETA_R = 3;
    vector<double> averages = {0.0, 0.0, 0.0, 0.0};
    vector<double> stddevs = {0.0, 0.0, 0.0, 0.0};

    // calculate average radius and angles for left lane markers
    for (const cv::Vec2d& line : lane_lines_left)
    {
        averages[RADIUS_L] += abs(line[0]);
        averages[THETA_L] += line[1];
    }

    averages[RADIUS_L] /= (double)lane_lines_left.size();
    averages[THETA_L] /= (double)lane_lines_left.size();

    // calculate the average radius and angles for the right lane markers
    for (const cv::Vec2d& line : lane_lines_right)
    {
        averages[RADIUS_R] += abs(line[0]);
        averages[THETA_R] += line[1];
    }

    averages[RADIUS_R] /= (double)lane_lines_right.size();
    averages[THETA_R] /= (double)lane_lines_right.size();

    // comput standard variance / deviation for the left sample
    for (const cv::Vec2d& line : lane_lines_left)
    {
        double radius_var2 = abs(line[0]) - averages[RADIUS_L];
        double theta_var2 = line[1] - averages[THETA_L];
        stddevs[RADIUS_L] += radius_var2 * radius_var2;
        stddevs[THETA_L] += theta_var2 * theta_var2;
    }

    stddevs[RADIUS_L] /= (double)lane_lines_left.size();
    stddevs[THETA_L] /= (double)lane_lines_left.size();

    // compute standard variance / deviation for the right sample
    for (const cv::Vec2d& line : lane_lines_right)
    {
        double radius_var2 = abs(line[0]) - averages[RADIUS_R];
        double theta_var2 = line[1] - averages[THETA_R];
        stddevs[RADIUS_R] += radius_var2 * radius_var2;
        stddevs[THETA_R] += theta_var2 * theta_var2;
    }

    stddevs[RADIUS_R] /= (double)lane_lines_right.size();
    stddevs[THETA_R] /= (double)lane_lines_right.size();
    
    vector<double> confidence = { 0.0, 0.0, 0.0, 0.0};
    
    const double P95_ANG_VARIANCE = (CV_PI * CV_PI / 4.0); // Absolute angular variance for the 95th percentile (4 sigma)
    const double P95_RAD_VARIANCE = 2 * (150.0 * 150.0); // Absolute radial variance for the 95th percentile (4 sigma)
    
    // compute the confidence of lane detection. Actual lane detections should
    // have few samples of lines and thus low variances. when variances approach
    // the maximum possible or reasonable variance, this should drop the confidence
    // to zero.
    confidence[RADIUS_L] = (P95_RAD_VARIANCE - 2.0 * stddevs[RADIUS_L]) / P95_RAD_VARIANCE;
    confidence[THETA_L] = (P95_ANG_VARIANCE - 2.0 * stddevs[THETA_L]) / P95_ANG_VARIANCE;
    confidence[RADIUS_R] = (P95_RAD_VARIANCE - 2.0 * stddevs[RADIUS_R]) / P95_RAD_VARIANCE;
    confidence[THETA_R] = (P95_ANG_VARIANCE - 2.0 * stddevs[THETA_R]) / P95_ANG_VARIANCE;
    
    if (confidence[RADIUS_L] < 0.0 || confidence[RADIUS_R] < 0.0)
    {
        return 0.0;
    }
    
    double min_confidence = 1.0;
    
    // always work with minimum confidenc
    for (int index = 0; index < 4; index++)
    {
        if (confidence[index] < min_confidence)
        {
            min_confidence = confidence[index];
        }
    }
    
    return min_confidence;
}

void LaneDetector::detect_lane()
{
    pthread_rwlock_rdlock(&imgproc_semaphore);

    if (!current_img)
    {
        pthread_rwlock_unlock(&imgproc_semaphore);
        printf("No image to process. Sleeping\n");
        std_msgs::ColorRGBA mesg;
        mesg.r = 0;
        mesg.b = 0;
        return;
    }

    // get the most recent image and allow for the background listener to overwrite the binding
    // this avoids blocking the listener while processing images
    cv::Mat* source_img = current_img;
    cv::Mat& img_color = *current_img;
    pthread_rwlock_unlock(&imgproc_semaphore);

    int hres = 1280;
    int vres = (int)((double)img_color.rows * ((double)hres / img_color.cols));
    cv::resize(img_color, img_color, cv::Size(hres, vres), cv::INTER_AREA);

    struct timeval now;
    gettimeofday(&now, NULL);
    unsigned long start_time = now.tv_sec * 1000 + now.tv_usec / 1000;

    // remove localized noise and unnecessary detail using median filter
    cv::medianBlur(img_color, img_color, median_blur_radius);

    // convert image to grayscale
    cv::Mat img_gray;
    cv::cvtColor(img_color, img_gray, cv::COLOR_BGR2GRAY);

    // perform canny edge detection
    cv::Mat edge_img;
    cv::Canny(img_gray, edge_img, canny_cont_thresh, canny_grad_thresh);

    // blot out top half of image
    cv::rectangle(edge_img,
                  cv::Point2i(0, 0),
                  cv::Point2i(img_gray.cols - 1, img_gray.rows / 3),
                  cv::Scalar(0.0),
                  cv::FILLED);

    vector<cv::Vec2d> lines;
    // 25.0 pix radius granularity, 1 deg angular granularity, 200 votes min for a line
    // 200 pixels min for a segment, up to 300 pixels between disconnected colinear segments
    cv::HoughLines(edge_img, lines, hough_radius_inc, hough_theta_inc, hough_min_votes);

    printf("Found %lu lines in the image\n", lines.size());
    vector<cv::Vec2d> raw_lines_left;
    vector<cv::Vec2d> raw_lines_right;
    vector<cv::Vec2d> lane_lines_left;
    vector<cv::Vec2d> lane_lines_right;

    for (auto& line : lines)
    {
        printf("  Radius: %f    Theta: %f\n", line[0], line[1] / CV_PI * 180.0);

        if ((line[1] > 7.0 * CV_PI / 180.0) && (line[1] < 173.0 * CV_PI / 180.0) &&
            ((line[1] < 8.0 * CV_PI / 18.0) || (line[1] > 10.0 * CV_PI / 18.0)))
        {
            double slope = -1.0 / tan(line[1]);
            double y_init = line[0] * sin(line[1]);
            double x_init = line[0] * cos(line[1]);
            double x1, y1, x2, y2 = 0.0;

            x1 = 0.0;
            y1 = -slope * x_init + y_init;

            if (slope < 0.0)
            {
                x2 = -y_init / slope + x_init;
                y2 = 0.0;
                lane_lines_left.push_back(cv::Vec2d(slope, -slope * x_init + y_init));
                raw_lines_left.push_back(line);
            }
            else
            {
                x2 = (double)edge_img.cols;
                y2 = slope * x2 - slope * x_init + y_init;
                lane_lines_right.push_back(cv::Vec2d(slope, -slope * x_init + y_init));
                raw_lines_right.push_back(line);
            }

            cv::line(edge_img,
                     cv::Point2i((int)x1, (int)y1),
                     cv::Point2i((int)x2, (int)y2),
                     cv::Scalar(255.0),
                     10);

            printf("  (%f, %f), (%f, %f)\n", x1, y1, x2, y2);
        }
    }


    if (!lane_lines_left.empty() && !lane_lines_right.empty())
    {
        cv::Vec2d lane_left(lane_lines_left[0][0], lane_lines_left[0][1]);
        cv::Vec2d lane_right(lane_lines_right[0][0], lane_lines_right[0][1]);

        for (int index = 1; index < lane_lines_left.size(); index++)
        {
            lane_left[0] += lane_lines_left[index][0];
            lane_left[1] += lane_lines_left[index][1];
        }
        
        lane_left[0] /= (double)lane_lines_left.size();
        lane_left[1] /= (double)lane_lines_left.size();

        for (int index = 1; index < lane_lines_right.size(); index++)
        {
            lane_right[0] += lane_lines_right[index][0];
            lane_right[1] += lane_lines_right[index][1];
        }
        
        lane_right[0] /= (double)lane_lines_right.size();
        lane_right[1] /= (double)lane_lines_right.size();

        int lane_start_y = edge_img.rows;
        int lane_left_start_x = ((double)lane_start_y - lane_left[1]) / lane_left[0];
        int lane_right_start_x = ((double)lane_start_y - lane_right[1]) / lane_right[0];
        int lane_center = lane_left_start_x + ((double)lane_right_start_x - lane_left_start_x) / 2.0;
        int xint = (lane_right[1] - lane_left[1]) / (lane_left[0] - lane_right[0]);
        int yint = lane_right[0] * xint + lane_right[1];

        cv::line(edge_img,
                 cv::Point2i(xint, yint),
                 cv::Point2i(lane_center, lane_start_y),
                 cv::Scalar(255.0),
                 5);

        printf("\n  Distance from Center: %d px\n"
               "  Right / Left X: %d, %d\n", edge_img.cols / 2 - lane_center, lane_right_start_x, lane_left_start_x);
        
        struct LanePose pose;
        pose.center_offset = edge_img.cols / 2 - lane_center;
        pose.heading = 0.0;
        pose.confidence = detection_confidence(raw_lines_left, raw_lines_right);
        current_pose = pose;

        printf("  Detection Confidence: %%%3.1f\n", pose.confidence * 100.0);
    }
    else
    {
        struct LanePose pose;
        pose.center_offset = 0;
        pose.heading = 0.0;
        pose.confidence = 0.0;
        current_pose = pose;
    }

    gettimeofday(&now, NULL);
    unsigned long end_time = now.tv_sec * 1000 + now.tv_usec / 1000;

    char filename[128];
    memset(filename, '\0', 128);

    sprintf(filename, "/media/nvidia/seniorDesign/LaneDetectionDebug/%lu_img.jpg", end_time);
    cv::imwrite(filename, img_color);

    memset(filename, '\0', 32);
    sprintf(filename, "/media/nvidia/seniorDesign/LaneDetectionDebug/%lu_edges.jpg", end_time);
    cv::imwrite(filename, edge_img);

    printf("Processing took: %lu msec\n\n----\n\n", end_time - start_time);

    std_msgs::ColorRGBA mesg;
    mesg.r = current_pose.center_offset / 850.0;
    if (mesg.r > 1.0)
    {
        mesg.r = 1.0;
    }
    else if (mesg.r < -1.0)
    {
        mesg.r = -1.0;
    }
    mesg.g = current_pose.confidence;
    pose_publisher.publish(mesg);

    delete source_img;
}

bool LaneDetector::set_median_blur_radius(int radius)
{
    if (radius % 2 == 1)
    {
        median_blur_radius = radius;
        return true;
    }

    return false;
}

void LaneDetector::set_hough_theta_inc(double degrees)
{
    hough_theta_inc = degrees * CV_PI / 180.0;
}

struct LanePose LaneDetector::get_vehicle_pose()
{
    return current_pose;
}

void LaneDetector::lane_guidance()
{
    struct pollfd stdin_timeout[1];
    stdin_timeout[0].fd = STDIN_FILENO;
    stdin_timeout[0].events = POLLIN | POLLPRI;

    while (ros::ok())
    {
        int poll_status = poll(stdin_timeout, 1, 10);

        if (poll_status > 0)
        {
            int ch = getchar();
            if ((char)ch == '\n' || (char)ch == '\r')
            {
                break;
            }
        }
        else
        {
            ros::spinOnce();
        }
    }
}
