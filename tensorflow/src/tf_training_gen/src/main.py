#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String, ColorRGBA
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

frame_count = 0
status = ColorRGBA(0,0,0,0)

def imageCallback(msg):
    global frame_count
    frame_count += 1
    if frame_count < 30:
        frame_count += 1
        return
    else:
        frame_count = 0

    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        print status
        # Save your OpenCV2 image as a jpeg
        cv2.imwrite(str(time.time())+'_'+str(status.r)+'_'+str(status.g)+'.jpeg', cv2_img)

def controlCallback(msg):
    global status
    msg_split = msg.data.split(',');
    steering = float(msg_split[0])
    throttle = float(msg_split[1])
    max_throttle = int(msg_split[2])
    steering = (steering - 1500.0) / 500.0
    throttle = (throttle - 1500.0) / max_throttle
    status = ColorRGBA(steering, throttle, 0, 0)
    print status

def main():
    rospy.init_node('tf_training')
    camSub = rospy.Subscriber("/camera/rgb/image_mono", Image, imageCallback)
    controlSub = rospy.Subscriber("robot_base_state", String, controlCallback)
    print 'test'
    rospy.spin()

if __name__ == '__main__':
    main()