#ifndef __ROVER_APP__
#define __ROVER_APP__

#include "RemoteCtlApp.h"
#include "ArduinoMessenger.h"
#include <string>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/ColorRGBA.h"

class RoverApp
{
    friend void* ctl_loop(void* rover_ptr);
    friend void* ros_publish_loop(void* rover_ptr);
    friend void* ros_listen_loop(void* rover_ptr);

private:
    bool running;
    bool autonomous; // take inputs from the ROS listener
    bool debug_out;

    ArduinoMessenger pwm_gateway;
    struct GamepadState gamepad_state;
    short steering_angle;
    short drive_power;
    short throttle_max;
    ros::Subscriber ctl_listener;

    int cmd_socket;
    int throttle_trim;
    struct sockaddr_in cmd_host_addr;

    pthread_t ctl_thread;
    pthread_t ros_publish_thread;
    pthread_t ros_listen_thread;
    pthread_mutex_t write_lock;
    pthread_rwlock_t rc_semaphore;
    pthread_rwlock_t base_state_semaphore;

    ros::NodeHandle rosnode;

public:
    RoverApp(const struct in_addr& host, bool debug_out);
    ~RoverApp();

    void recieve_cmds();
};

#endif
