#! /usr/bin/env python

import rospy
import cv2
from std_msgs.msg import Header
from std_msgs.msg import Int32MultiArray, Bool
from sensor_msgs.msg import Image


def bool_callback(data):
    return 0

def color_camera_calback(data):
    return 0

def depth_camera_callback(data):
    return 0

# subscriber to the camera
# make a subscriber to the topic that tells us that we are in bowl mode
# make a publisher that publishes position of bowl.
pub = rospy.Publisher('/bowl_cords', Int32MultiArray, queue_size=1)
sub_bool = rospy.Subscriber("/find_bowl_trigger", Bool, bool_callback)
color_sub = rospy.Subscriber("/color/image_raw", Image, color_camera_calback)
depth_sub = rospy.Subscriber("/depth/image_raw", Image, depth_camera_callback)


if __name__ == '__main__':
    rospy.init_node('moveit_node')

    print("This works!")

    rospy.spin()