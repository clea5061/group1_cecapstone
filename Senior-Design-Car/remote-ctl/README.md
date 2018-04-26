# Base Control Module

This is the module for controlling the steering and throttle of the car either
by RC over Wifi or autonomously with ROS. This module has two parts:

1. The base station for teleoperated control via Xbox controller.
2. The ROS node on the Jetson for controlling the steering and motors.

BOTH parts must be running in order for the car to be controlled. This is for
safety reasons.


## To Build:

**Build the base station executable:**

    $ cd src/base-ctl/
    $ ./build-base-station.sh

**Build the ROS node for base control:**

    $ catkin_make


## To Install / Run:

### Raspberry Pi Controller

The Raspberry Pi in the lab is set up to run the 'base-station' server
automatically at boot time for controlling the car via Xbox controller over Wifi.
If the server has been updated, you will need to reinstall it.

    $ ssh pi@192.168.1.6          # ssh into Raspberry Pi. password 'seniordesign2018'
    $ cd git/Senior-Design-Car/
    $ git pull                    # update repository

Then follow build instructions above. Once built:

    $ cp base-station /usr/bin/
    $ cd ../../
    $ cp base-station.service $(pkg-config systemd --variable=systemdsystemconfdir)
    $ sudo systemctl restart base-station

Note that you will need to have the Xbox controller attached to the Pi at all
times, else the server will fail to start. If it does fail to start, then it can
be started using:

    $ sudo systemctl start base-station

While ssh'd into the Pi.


### To start the ROS node for controlling the car:

**THE CAR WILL NOT MOVE AT ALL IF YOU DO NOT RUN THIS ROS NODE**

Either from the Jetson desktop or while ssh'd into it, run:

    $ devel/lib/base-ctl/base-ctl_node 192.168.1.6

While in this folder. Note that this process currently cannot be terminated via
Ctl+C so you will need to run:

    $ kill -s SIGTERM $(pgrep base-ctl_node)

From another console in order to stop it.


## Control Scheme

    Left Stick X   - Steering
    Right Stick Y  - Throttle
    B              - Emergency Stop / Cancel Autonomous
    Start & Back   - Change to Autonomous Control (must be pressed simultaneously)

## Dependencies

You will need

- SDL2:
  https://libsdl.org

    $ sudo apt-get install libsdl2_2.0-0
    $ sudo apt-get install libsdl2-dev
