#!/bin/bash

cd /home/nvidia/jetsonbot
source devel/setup.bash
rosrun xv_11_laser_driver laserScanner.py
