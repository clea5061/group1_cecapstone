#!/usr/bin/env python

import os
import rospy
import time
import tensorflow as tf
import numpy as np
from threading import Lock
from car_cnn import cnn_car
from lidar_cnn import lidar_car
from sensor_msgs.msg import Image
from std_msgs.msg import String, ColorRGBA, Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2

def build_graph():
    gi = tf.Graph()
    return gi


bridge = CvBridge()
vehMsg = ColorRGBA(0, 0, 0, 0)
gc = build_graph()
gl = build_graph()
sess= None
sess2= None
camera_model= None
lidar_model= None
neural_link = False
lidar_weight_max = .8	#change this to manipulate how much lidar/camera influence driving (0-1)
lidar_update_freq = 400.0 	#corresponds to lidar @5Hz*2
last_lidar = None
lidar_lock = Lock()
camera_lock = Lock()
lidar_steer = 0.0
camera_steer = 0.0

def init_networks():
    global sess
    global sess2
    global camera_model
    global lidar_model
    sess = tf.InteractiveSession(graph = gc)
    with gc.as_default():
        camera_model = cnn_car()
        saver = tf.train.Saver()
        saver.restore(sess, "/media/deepLearn/training_data/brains/camera/model.ckpt")
    sess2 = tf.InteractiveSession(graph = gl)
    with gl.as_default():
        lidar_model = lidar_car();
        saver = tf.train.Saver()
        saver.restore(sess2, "/media/deepLearn/training_data/brains/lidar/model.ckpt")


def createVehicleRequest(throttle):
    global lidar_steer
    global camera_steer
    with lidar_lock:
        lidar_weight = lidar_weight_max - (rospy.get_rostime()-last_lidar).to_nsec()/1000000.0/lidar_update_freq  #Times arent right, but data types work
        camera_weight = 1 - lidar_weight
        print "L: "+str(lidar_weight)
        print "C: "+ str(camera_weight)
        with camera_lock:
                steer = camera_steer*camera_weight + lidar_steer*lidar_weight
    vehMsg.r = throttle
    vehMsg.g = steer
    vehMsg.a = 1.0 # A as 1 indicates CNN
    #print vehMsg
    return vehMsg

def imageCallback(msg, publisher):
    global neural_link
    global camera_steer
    if neural_link:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2_img = cv2.resize(cv2_img, (200, 150), interpolation=cv2.INTER_CUBIC)
        cv2_img = cv2_img[35:,:,:]
        normed_img = cv2_img.astype(dtype=np.float32) / 255.0
        # normed_img = np.reshape(normed_img, (115, 200, 1))
        with camera_lock:
            camera_steer = camera_model.y_out.eval(session=sess, feed_dict={camera_model.x: [normed_img],
                                                                    camera_model.keep_prob_fc1: 1.0,
                                                                    camera_model.keep_prob_fc2: 1.0,
                                                                    camera_model.keep_prob_fc3: 1.0,
                                                                    camera_model.keep_prob_fc4: 1.0})[0][0]
        publisher.publish(createVehicleRequest(0))

    #Tyler and Tim don't know python especially on ROS, so fix this shit
	#ros::Time:now() should give current time stamp
	#lidarmessage.header.stamp.nsec() should give time of last lidar message from ROS timestamp

    #end shit fixing

def lidarCallback(msg, publisher):
    global neural_link
    global lidar_steer
    global last_lidar
    if neural_link:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2_img = cv2_img[:, :, :1]
        normed_img = cv2_img.astype(dtype=np.float32) / 255.0
        with lidar_lock:
            lidar_steer = lidar_model.y_out.eval(session=sess2, feed_dict={lidar_model.x: [normed_img],
                                                                lidar_model.keep_prob_fc1: 1.0,
                                                                lidar_model.keep_prob_fc2: 1.0})[0][0]
            last_lidar = rospy.get_rostime()
        publisher.publish(createVehicleRequest(0))

def neuralCallback(msg):
    global neural_link
    neural_link = msg.data



def main():
    global last_lidar
    rospy.init_node('cnn_driving_node')
    last_lidar = rospy.get_rostime()
    print "Loading CNN Model"
    init_networks()
    print "Model Loaded"
    req = rospy.Publisher("/robot_base_control", ColorRGBA, queue_size=10)
    camSub = rospy.Subscriber("/camera/rgb/image_mono", Image, imageCallback, req)
    lidarSub = rospy.Subscriber("/lidargrid", Image, lidarCallback, req) #check declaration
    neuralSub = rospy.Subscriber("neural_link", Bool, neuralCallback)
    rospy.spin()

if __name__ == '__main__':
    main()
