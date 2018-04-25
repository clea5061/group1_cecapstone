# Start ROS
roscore &

sleep 5

# Start remote-ctl
~/src/git/Senior-Design-Car/remote-ctl/devel/lib/base-ctl/base-ctl_node 192.168.8.158 > /dev/null &

# Start laserscan publisher
rosrun xv_11_laser_driver neato_laser_publisher &

# Start realsense camera
roslaunch realsense_camera r200_depth.launch &

# Start webcam
#python ../Webcam/Webcam.py &

# Start lanecentering application
##rosrun xv_11_laser_driver 2laserScanner.py
##rosrun xv_11_laser_driver laneCentering.py
##rosrun xv_11_laser_driver wallFollow.py
##rosrun xv_11_laser_driver cornering.py
rosrun xv_11_laser_driver master.py
