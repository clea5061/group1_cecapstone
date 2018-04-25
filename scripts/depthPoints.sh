# Start ROS
roscore &

sleep 5

# Start remote-ctl
#~/src/git/Senior-Design-Car/remote-ctl/devel/lib/base-ctl/base-ctl_node 192.168.8.158 > /dev/null &

# Start camera publisher
roslaunch realsense_camera r200_nodelet_rgbd.launch &

# Start rviz
rosrun rviz rviz -d ../jetsonbot/src/realsense/realsense_camera/rviz/realsense_rgbd_pointcloud.rviz
