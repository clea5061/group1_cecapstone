import rospy
import time
from sensor_msgs.msg import Image


def imageCallback(msg):
    f = open(str(time.time())+".jpg", "w")
    f.write(msg.data)
    f.flush()
    f.close()

def main():
    rospy.init_node('tf_training')
    camSub = rospy.Subscriber("/camera/rgb/image_raw", Image, imageCallback)
