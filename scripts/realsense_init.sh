#!/bin/bash

cd /home/nvidia/jetsonbot
source devel/setup.bash
roscd realsense_camera
roslaunch realsense_camera r200_nodelet_rgbd.launch
