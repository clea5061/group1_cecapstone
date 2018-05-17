#include "RoverApp.h"
#include <exception>
#include <stdexcept>
#include <stropts.h>
#include <poll.h>

using namespace std;

double steering_input;
double throttle_input;
double cnn_input;

const char* err_mesgs[] = {
    "No route to requested IP address",
    "Connection refused",
    "Connection timed out",
    "Connect was reset",
    "Unhandled connection error",
};

const char* connect_err_mesg(int code)
{
    const char* err_mesg = NULL;

    switch (code)
    {
    case EADDRNOTAVAIL:
        err_mesg = err_mesgs[0];
        break;

    case ECONNREFUSED:
        err_mesg = err_mesgs[1];
        break;

    case ENETUNREACH:
        err_mesg = err_mesgs[0];
        break;

    case ETIMEDOUT:
        err_mesg = err_mesgs[2];
        break;

    case ECONNRESET:
        err_mesg = err_mesgs[3];
        break;

    default:
        err_mesg = err_mesgs[4];
        break;
    }

    return err_mesg;
}

void base_control_listener(const std_msgs::ColorRGBA& throttle_steer)
{
    throttle_input = (double)throttle_steer.r;
    steering_input = (double)throttle_steer.g;
    cnn_input = (double)throttle_steer.a;
}

void* ctl_loop(void* rover_ptr)
{
    RoverApp* rover = (RoverApp*)rover_ptr;

    pthread_rwlock_rdlock(&rover->rc_semaphore);
    bool running = rover->running;
    pthread_rwlock_unlock(&rover->rc_semaphore);
	ros::Publisher tf_publisher = rover->rosnode.advertise<std_msgs::Bool>("tensor_flow_training", 4);
	ros::Publisher cnn_publisher = rover->rosnode.advertise<std_msgs::Bool>("neural_link", 4);

    short steering_angle = 1500;
    short drive_power = 1500;

    struct timeval systime;
    gettimeofday(&systime, NULL);
    long start_time = systime.tv_sec * 1000000 + systime.tv_usec;
    int update_div = 0; // clock divider to avoid flooding Arduino
    int estop_expire = -1;

    bool speed_limit_up = false;
    bool speed_limit_down = false;
	bool tf_training_toggle = false;
	bool tf_training_state = false;
    double throttle_max = 100;
    double r_trigger = 0.0D;
    double l_trigger = 0.0D;

    while (running)
    {
        pthread_rwlock_rdlock(&rover->rc_semaphore);
        struct GamepadState gamepad_state = rover->gamepad_state;
        running = rover->running;
        pthread_rwlock_unlock(&rover->rc_semaphore);

        // if throttle limit up button bumped
        if (gamepad_state.button[RB_BTN])
        {
            speed_limit_up = true;
        }
        else if (speed_limit_up)
        {
            speed_limit_up = false;
            if (throttle_max < 500.0)
            {
                rover->throttle_trim = throttle_max;
                throttle_max += 12.5;
            }
        }

	if (gamepad_state.button[X_BTN])
	{
		tf_training_toggle = true;
	} else if (tf_training_toggle) {
		tf_training_toggle = false;
		tf_training_state = !tf_training_state;
		std_msgs::Bool mesg;
       		mesg.data = tf_training_state;
		tf_publisher.publish(mesg);
	}

        // if throttle limit down button bumped
        if (gamepad_state.button[LB_BTN])
        {
            speed_limit_down = true;
        }
        else if (speed_limit_down)
        {
            speed_limit_down = false;
            if (throttle_max > 0.0)
            {
                rover->throttle_trim = throttle_max;
                throttle_max -= 12.5;
            }
        }

        // if E Stop button pressed
        if (gamepad_state.button[B_BTN])
        {
            estop_expire = update_div + 100; // cut input for roughly 1 second
            rover->autonomous = false;
		std_msgs::Bool mesg;
       		mesg.data = rover->autonomous;
		cnn_publisher.publish(mesg);
        }

        // if user wanted car to go into autonomous mode
        if (gamepad_state.button[START_BTN] && gamepad_state.button[BACK_BTN] && !rover->autonomous)
        {
            rover->autonomous = true;
		std_msgs::Bool mesg;
		mesg.data = rover->autonomous;
		cnn_publisher.publish(mesg);
        }

        steering_angle = 1500;
        drive_power = 1500;

        if (update_div > estop_expire)
        {
            double throttle_val = 0.0;
            double steering_val = 0.0;
	        double net_throttle = 0.0;
	        r_trigger = (gamepad_state.axis_rt + 1.0d) / 2.0D; //Forward, default state is -1.00
	        l_trigger = (gamepad_state.axis_lt + 1.0d) / 2.0D; // Reverse, default state is -1
		
	        net_throttle = r_trigger-l_trigger;
            if (abs(gamepad_state.axis_lx) > 0.2)
            {
                steering_val = gamepad_state.axis_lx;

                if (steering_val < 0.0)
                {
                    steering_val = (steering_val + 0.2) / 0.8;
                }
                else
                {
                    steering_val = (steering_val - 0.2) / 0.8;
                }
            }
	

            if (abs(net_throttle) > 0.3)
            {
                throttle_val = net_throttle;

                if (throttle_val < 0)
                {
                    throttle_val = (throttle_val + 0.3) / 0.7;
                }
                else
                {
                    throttle_val = (throttle_val - 0.3) / 0.7;
                }
            }

            if (rover->autonomous)
            {
		if(cnn_input < 0.1) {
                    throttle_val = throttle_input;
		}
                steering_val = steering_input;
            }

            steering_angle = (short)(steering_val * 500.0 + 1500.0);
            drive_power = (short)(throttle_val * throttle_max + 1500.0);
        }

        if (update_div % 10 == 0)
        {
            rover->pwm_gateway.send_mesg(0, (void*)&drive_power, sizeof(short));
            rover->pwm_gateway.send_mesg(1, (void*)&steering_angle, sizeof(short));
        }

        pthread_rwlock_wrlock(&rover->base_state_semaphore);
        rover->steering_angle = steering_angle;
        rover->drive_power = drive_power;
	rover->throttle_max = throttle_max;
        pthread_rwlock_unlock(&rover->base_state_semaphore);

        gettimeofday(&systime, NULL);
        long sleep_time = 10 * 1000 - (systime.tv_sec * 1000000 + systime.tv_usec - start_time);

        if (sleep_time > 0)
        {
            usleep(sleep_time);
        }

        gettimeofday(&systime, NULL);
        start_time = systime.tv_sec * 1000000 + systime.tv_usec;
        update_div++;
    }

    steering_angle = 1500;
    drive_power = 1500;

    rover->pwm_gateway.send_mesg(0, (void*)&drive_power, sizeof(short));
    rover->pwm_gateway.send_mesg(1, (void*)&steering_angle, sizeof(short));

    return NULL;
}

void* ros_publish_loop(void* rover_ptr)
{
    RoverApp* rover = (RoverApp*)rover_ptr;
    ros::Publisher base_publisher = rover->rosnode.advertise<std_msgs::String>("robot_base_state", 4);

    pthread_rwlock_rdlock(&rover->rc_semaphore);
    bool running = rover->running;
    pthread_rwlock_unlock(&rover->rc_semaphore);

    while (running && ros::ok())
    {
        pthread_rwlock_rdlock(&rover->base_state_semaphore);
        short steering_angle = rover->steering_angle;
        short drive_power = rover->drive_power;
	short throttle_max = rover->throttle_max;
        pthread_rwlock_unlock(&rover->base_state_semaphore);

        string mesg_data(to_string(steering_angle));
        mesg_data += ",";
        mesg_data += to_string(drive_power);
	mesg_data += ",";
	mesg_data += to_string(throttle_max);

        std_msgs::String mesg;
        mesg.data = mesg_data;

        base_publisher.publish(mesg);

        usleep(100 * 1000);

        pthread_rwlock_rdlock(&rover->rc_semaphore);
        running = rover->running;
        pthread_rwlock_unlock(&rover->rc_semaphore);
    }

    return NULL;
}

void* ros_listen_loop(void* rover_ptr)
{
    RoverApp* rover = (RoverApp*)rover_ptr;

    pthread_rwlock_rdlock(&rover->rc_semaphore);
    bool running = rover->running;
    pthread_rwlock_unlock(&rover->rc_semaphore);

    while (running)
    {
        ros::spinOnce();

        usleep(1000);

        pthread_rwlock_rdlock(&rover->rc_semaphore);
        running = rover->running;
        pthread_rwlock_unlock(&rover->rc_semaphore);
    }

    return NULL;
}

RoverApp::RoverApp(const struct in_addr& host, bool _debug_out) : rosnode(ros::NodeHandle()), debug_out(_debug_out)
{
    printf("Creating TCP socket for recieving commands...\n");
    cmd_socket = socket(AF_INET, SOCK_STREAM, 0);

    if (cmd_socket < 0)
    {
        throw runtime_error(string("Failed to create TCP socket. socket() error ") + to_string(errno));
    }

    memset(&cmd_host_addr, 0, sizeof(struct sockaddr_in));
    cmd_host_addr.sin_family = AF_INET;         // internet type socket
    cmd_host_addr.sin_port = htons(5309);       // port 5309. need to use correct byte order
    cmd_host_addr.sin_addr.s_addr = host.s_addr;

    printf("Attempting to connect to control host at %s:5309...\n", inet_ntoa(cmd_host_addr.sin_addr));

    if (connect(cmd_socket, (struct sockaddr*)&cmd_host_addr, sizeof(struct sockaddr_in)) == -1)
    {
        close(cmd_socket);
        throw runtime_error(string("Failed to connect to control host. connect() error ") + connect_err_mesg(errno));
    }

    printf("Success! Now listening for commands...\n");
    memset((void*)&gamepad_state, 0, sizeof(struct GamepadState));

    if (pthread_rwlock_init(&rc_semaphore, NULL) ||
        pthread_rwlock_init(&base_state_semaphore, NULL))
    {
        close(cmd_socket);
        throw runtime_error("Failed to create guarding semaphores");
    }

    running = true;

    if (pthread_create(&ctl_thread, NULL, &ctl_loop, (void*)this) == -1)
    {
        close(cmd_socket);
        throw runtime_error(string("Failed to create control thread. pthread_create() error ") + to_string(errno));
    }

    if (pthread_create(&ros_publish_thread, NULL, &ros_publish_loop, (void*)this) == -1)
    {
        pthread_rwlock_wrlock(&rc_semaphore);
        running = false;
        pthread_rwlock_unlock(&rc_semaphore);
        pthread_join(ctl_thread, NULL);

        close(cmd_socket);
        pthread_rwlock_destroy(&rc_semaphore);
        pthread_rwlock_destroy(&base_state_semaphore);

        throw runtime_error(string("Failed to create ROS publisher thread. pthread_create() error ") + to_string(errno));
    }

    ctl_listener = rosnode.subscribe("robot_base_control", 5, base_control_listener);

    if (pthread_create(&ros_listen_thread, NULL, &ros_listen_loop, (void*)this) == -1)
    {
        pthread_rwlock_wrlock(&rc_semaphore);
        running = false;
        pthread_rwlock_unlock(&rc_semaphore);
        pthread_join(ctl_thread, NULL);
        pthread_join(ros_publish_thread, NULL);

        close(cmd_socket);
        pthread_rwlock_destroy(&rc_semaphore);
        pthread_rwlock_destroy(&base_state_semaphore);

        throw runtime_error(string("Failed to create ROS listener thread. pthread_create() error ") + to_string(errno));
    }
}

RoverApp::~RoverApp()
{
    pthread_rwlock_wrlock(&rc_semaphore);
    running = false;
    pthread_rwlock_unlock(&rc_semaphore);

    pthread_join(ctl_thread, NULL);
    pthread_join(ros_publish_thread, NULL);
    pthread_join(ros_listen_thread, NULL);

    // ensure the car stops!!!
    short servo_neutral = 1500;
    pwm_gateway.send_mesg(0, (void*)&servo_neutral, sizeof(short));
    pwm_gateway.send_mesg(1, (void*)&servo_neutral, sizeof(short));

    pthread_rwlock_destroy(&rc_semaphore);
    pthread_rwlock_destroy(&base_state_semaphore);
    close(cmd_socket);
}

bool verify_mesg(const char* buf, int size)
{
    char checksum = 0;
    size++; // include checksum byte

    for (int index = 0; index < size; index++)
    {
        checksum += buf[index];
    }

    return checksum == '\0';
}

void RoverApp::recieve_cmds()
{
    struct pollfd recv_timeout;
    recv_timeout.fd = cmd_socket;
    recv_timeout.events = POLLIN | POLLPRI;
    int timeout_ms = 1500;
    char mesg[DEFAULT_MESG_SIZE];
    int bytes_recieved = 1;

    while (bytes_recieved > 0 && ros::ok())
    {
        memset((void*)mesg, 0, DEFAULT_MESG_SIZE);
        int poll_status = poll(&recv_timeout, 1, timeout_ms);
        if (poll_status > 0)
        {
            bytes_recieved = recv(cmd_socket, (void*)mesg, DEFAULT_MESG_SIZE, 0);

            if (bytes_recieved == -1)
            {
                if ((errno == EAGAIN) || (errno == EWOULDBLOCK))
                {
                    continue;
                }
                else if (errno == ECONNREFUSED)
                {
                    printf("\nLost connection to host. Stopping...\n");
                    close(cmd_socket);
                }
                else
                {
                    printf("\nConnection error. recv() error %d\n", errno);
                    close(cmd_socket);
                }
            }
            else if (bytes_recieved == 0)
            {
                printf("\nHost at %s closed connection. Stopping...\n", inet_ntoa(cmd_host_addr.sin_addr));
                shutdown(cmd_socket, SHUT_RDWR);
                close(cmd_socket);
            }
            else if (bytes_recieved != DEFAULT_MESG_SIZE)
            {
                continue; // bad packet. skip
            }
            else
            {
                if (!verify_mesg(mesg, sizeof(struct GamepadState)))
                {
                    continue; // bad packet. skip
                }

                pthread_rwlock_wrlock(&rc_semaphore);
                memcpy((void*)&gamepad_state, (void*)mesg, sizeof(struct GamepadState));
                pthread_rwlock_unlock(&rc_semaphore);

                char button[33];
                memset((void*)button, 0, 33);

                for (int index = 0; index < 32; index++)
                {
                    button[index] = gamepad_state.button[index] + 0x30;
                }

                if (!debug_out)
                {
                    printf("LX: % 6.04f   LY: % 6.04f   LT: % 6.04f   RX: % 6.04f   RY: % 6.04f   RT: % 6.04f   Trim: %d   %s\r",
                           gamepad_state.axis_lx, gamepad_state.axis_ly, gamepad_state.axis_lt,
                           gamepad_state.axis_rx, gamepad_state.axis_ry, gamepad_state.axis_rt,
                           throttle_trim,
                           button);
                }
            }
        }
        else
        {
            printf("\nRecieving gamepad state timed out. Stopping\n");
            bytes_recieved = -1; // force exit
        }
    }

    if (!ros::ok())
    {
        printf("\nROS core encountered problem. Unsafe to continue. Stopping\n");
    }
}
