# Use jetsonbot to start ros and lidar
source /home/nvidia/jetsonbot/devel/setup.bash

# Start remote-ctl
#~/src/git/Senior-Design-Car/remote-ctl/devel/lib/base-ctl/base-ctl_node 192.168.1.4 > /dev/null &

# Start ros and lidar
roscore &
rosrun xv_11_laser_driver neato_laser_publisher &

# Wait to make sure roscore started
sleep 5

# Use kinetic to start hector
source /opt/ros/kinetic/setup.bash

# Start imu
#roslaunch razor_imu_9dof razor-pub.launch &

#sleep 10

# Start imu tf transform
#roslaunch hector_imu_attitude_to_tf example.launch &

# Start hector
rosrun tf static_transform_publisher 0 0 0 0 0 0 map scanmatcher_frame 10 &
roslaunch hector_slam_launch neato.launch &

# Start rviz
rosrun rviz rviz -d /home/nvidia/jetsonbot/src/hector_slam-catkin/hector_slam_launch/rviz_cfg/hector_slam.rviz
