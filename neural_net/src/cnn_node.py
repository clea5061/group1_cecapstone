
import os
import rospy
import time
import tensorflow as tf
import numpy as np
from car_cnn import cnn_car
from sensor_msgs.msg import Image
from std_msgs.msg import String, ColorRGBA, Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()
vehMsg = ColorRGBA(0, 0, 0, 0)
sess = tf.Session()
saver = tf.train.Saver()
model = cnn_car()
neural_link = False

def createVehicleRequest(throttle, steer):
    vehMsg.r = throttle
    vehMsg.g = steer
    vehMsg.a = 1.0 # A as 1 indicates CNN
    return vehMsg

def imageCallback(msg, publisher):
    global neural_link
    if neural_link:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2_img = cv2.resize(cv2_img, (200, 150), interpolation=cv2.INTER_CUBIC)
        cv2_img = cv2_img[35:,:,:]
        normed_img = cv2_img.astype(dtype=np.float32) / 255.0
        # normed_img = np.reshape(normed_img, (115, 200, 1))
        steer = model.y_out.eval(session=sess, feed_dict={model.x: [normed_img],
                                                                    model.keep_prob_fc1: 1.0,
                                                                    model.keep_prob_fc2: 1.0,
                                                                    model.keep_prob_fc3: 1.0,
                                                                    model.keep_prob_fc4: 1.0})
        publisher.publish(createVehicleRequest(0,steer))

def neuralCallback(msg):
    global neural_link
    neural_link = msg.data

def main():
    rospy.init_node('cnn_driving_node')
    print "Loading CNN Model"
    saver.restore(sess, "/media/deepLearn/training_data/train_set/model.ckpt")
    req = rospy.Publisher("/robot_base_control", ColorRGBA, queue_size=10)
    camSub = rospy.Subscriber("/camera/rgb/image_mono", Image, imageCallback, req)
    neuralSub = rospy.Subscriber("neural_link", Bool, neuralCallback)
    rospy.spin()

if __name__ == '__main__':
    main()
