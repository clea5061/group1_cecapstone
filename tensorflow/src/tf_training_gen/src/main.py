#!/usr/bin/env python

import os
import rospy
import time
from threading import Lock
from sensor_msgs.msg import Image
from std_msgs.msg import String, ColorRGBA, Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

train_data_dir = "/media/deepLearn/training_data/"

class trainGen:

    def __init__(self):
        self.frame_count = 0
        self.active_run = 0
        self.output_train_data = False
        self.status = ColorRGBA(0,0,0,0)
        self.status_lock = Lock()
        try:
            os.mkdir(train_data_dir)
        except OSError:
            pass  # do nothing

        rospy.init_node('tf_training')
        camSub = rospy.Subscriber("/camera/rgb/image_mono", Image, self.imageCallback)
        lidSub = rospy.Subscriber("/lidargrid", Image, self.lidarGridCallback)
        controlSub = rospy.Subscriber("robot_base_state", String, self.controlCallback)
        trainSub = rospy.Subscriber("tensor_flow_training", Bool, self.trainingCallback)
        print "Training Initialized"
        rospy.spin()

    def imageCallback(self, msg):
        if self.output_train_data:
            seq = msg.header.seq
            stamp = msg.header.stamp
            self.frame_count += 1
            if self.frame_count < 1:
                return
            else:
                self.frame_count = 0

            #rospy.loginfo("Image Received")
            try:
                # Convert your ROS Image message to OpenCV2
                cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
                print self.status
                # Save your OpenCV2 image as a jpeg
                with self.status_lock:
                    cv2.imwrite(train_data_dir +str(self.active_run) + '/camera/'+ str(seq) +'_' + str(stamp) + '_' + str(self.status.r) + '_' + str(self.status.g) + '.jpeg', cv2_img)
            except CvBridgeError as e:
                print(e)

    def lidarGridCallback(self, msg):
        if self.output_train_data:
            seq = msg.header.seq
            stamp =msg.header.stamp
            try:
                cv2_img = bridge.imgmsg_to_cv2(msg, "mono8")
                with self.status_lock:
                    cv2.imwrite(train_data_dir + str(self.active_run) + '/lidar/' + str(seq) + '_' + str(stamp) + '_' + str(self.status.r) + '_' + str(self.status.g) + '.jpeg', cv2_img)
            except CvBridgeError as e:
                print(e)


    def controlCallback(self, msg):
        msg_split = msg.data.split(',')
        steering = float(msg_split[0])
        throttle = float(msg_split[1])
        max_throttle = int(msg_split[2])
        steering = (steering - 1500.0) / 500.0
        throttle = (throttle - 1500.0) / max_throttle
        with self.status_lock:
            self.status = ColorRGBA(steering, throttle, 0, 0)
        # print status

    def trainingCallback(self, msg):
        self.active_run = time.time()
        self.output_train_data = msg.data
        if self.output_train_data:
            try:
                os.mkdir(train_data_dir + str(self.active_run) + '/')
                os.mkdir(train_data_dir + str(self.active_run) + '/camera/')
                os.mkdir(train_data_dir + str(self.active_run) + '/lidar/')
            except OSError:
                pass  # do nothing
        print "Training is: " + "Enabled" if self.output_train_data else "Disabled"



if __name__ == '__main__':
    trainGen()