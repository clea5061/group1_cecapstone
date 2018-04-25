# Start ROS
startRos(){
    roscore &
    var+=($!)
    sleep 5
}

# Start remote-ctl
startController(){
    ~/src/git/Senior-Design-Car/remote-ctl/devel/lib/base-ctl/base-ctl_node 192.168.8.158 > /dev/null &
    var+=($!)
}

# Start laserscan publisher
startLidar(){
    rosrun xv_11_laser_driver neato_laser_publisher &
    var+=($!)
}

# Start realsense camera
startDepthSense(){
    roslaunch realsense_camera r200_depth.launch &
    var+=($!)
}

# Start webcam
startWebcam(){
    python Webcam.py $
    var+=($!)
}

# Start lanecentering application
startAuton(){
    ##rosrun xv_11_laser_driver 2laserScanner.py
    ##rosrun xv_11_laser_driver laneCentering.py
    ##rosrun xv_11_laser_driver wallFollow.py
    rosrun xv_11_laser_driver cornering.py
    ##rosrun xv_11_laser_driver cameraCloud.py
    var+=($!)
}

sleepInfinity(){
    echo "Killing child processes"
    pkill -P $$
    #kill -- -$$
}

# BEGIN SCRIPT
startRos

while getopts "cldwa:" OPTION
do
    case $OPTION in
        c) 
            echo "Starting X-Box Controller Application"
            startController
            continue
            ;;
        l)
            echo "Starting Lidar ROS Publisher"
            startLidar
            continue
            ;;
        d)
            echo "Starting DepthSense ROS Publisher"
            startDepthSense
            continue
            ;;
        w)
            echo "Starting Webcam Recording Application"
            startWebcam
            continue
            ;;
        a)
            echo "Starting Autonomous Application"
            MYOPTF="$MYOPTF $OPTARG"
            echo MYOPTF
            startAuton
            continue
            ;;
    esac
done

trap exitScript SIGINT SIGTERM

sleep infinity

echo $var
