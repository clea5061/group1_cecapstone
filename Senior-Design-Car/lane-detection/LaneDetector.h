#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <sys/time.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

struct LanePose
{
    int center_offset; ///< car's offset from center in pixels
    double heading;    ///< car's deviation from straight in radians
    double confidence; ///< confidence that a lane has actually be detected
};

class LaneDetector
{
    friend void* lane_detection_loop(void* detector_ptr);

private:
    bool running;
    int median_blur_radius; ///< radius used for median filtering. must be odd so it is kept private
    struct LanePose current_pose;

    ros::NodeHandle rosnode;
    ros::Subscriber laneimg_listener;

    pthread_t lane_detection_thread;
    pthread_rwlock_t exit_semaphore;

    void detect_lane();

public:
    int canny_grad_thresh; ///< gradient threshold needed to start a canny edge
    int canny_cont_thresh; ///< gredient threshold needed to continue a canny edge
    int hough_radius_inc;  ///< radius step size for Hough transform
    double hough_theta_inc; ///< theta step size for Hough transform
    int hough_min_votes;   ///< minimum number of votes needed to detect a Hough line

    LaneDetector();
    ~LaneDetector();

    bool set_median_blur_radius(int radius);
    void set_hough_theta_inc(double inc);
    struct LanePose get_vehicle_pose();
    void lane_guidance();
};
