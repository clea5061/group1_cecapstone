1/10 Scale Autonomous RC Car
===

Compiling
---
NeuralNet
```
cd neural_net
catkin_make
echo "source $PWD/devel/setup.sh" >> ~/.bashrc
```
Remote Control
```
cd Senior-Design-Car/remote-ctl
catkin_make
echo "source $PWD/devel/setup.sh" >> ~/.bashrc
```
Tensorflow
```
cd tensorflow
catkin_make
echo "source $PWD/devel/setup.sh" >> ~/.bashrc
```

Startup procedure
---

ROS
```
roscore &
```
ROS Nodes

Lidar
```
rosrun xv_11_laser_driver neato_laser_publisher
```

Real Sense Camera
```
roslaunch realsense_camera r200_depth.launch
```

Controller
```
./Senior-Design-Car/remote-ctl/devel/lib/base-ctrl/base-ctrl-node <ip-address of pi>
```

Fused Neural Network
```
cd neural_net
python cnn_node_master.py
```

Camera Neural Network
```
cd neural_net
python cnn_node.py
```

Controls
---
| Button           | Action   |
|------------------|----------|
| B                | Emergency Brake/Disable Neural Network      |
| X                | Enable/Disable Training      |
| Left-Stick       | Steering     |
| Right-Trigger    | Accelerate     |
| Left-Trigger     | Reverse     |
| Start & Back     | Enable Neural Network |
| Right-Bumper     | Increase Max Throttle |
| Left-Bumper      | Decrease Max Throttle |


Important Directories
---
Training Data Folder: `/media/deepLearn/training_data`

Trained Models: `/media/deepLearn/training_data/brains/{camera/lidar}`
