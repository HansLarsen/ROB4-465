#! /usr/bin/env python

import rospy
import cv2
from std_msgs.msg import Header
from std_msgs.msg import Int32MultiArray, Bool
from sensor_msgs.msg import Image
from from cv_bridge import CvBridge

def bool_callback(data):
    return data

def color_camera_calback(data):
    try:
        rgb_image = CvBridge().imgmsg_to_cv2(image_message, desired_encoding = "rgb8")

    except CvBridgeError as e:
    print(e)

    cv2.imshow("rgb" , rgb_image)
    

def depth_camera_callback(data):
    return 0

def find_cordinate():
    ## code for finding blob cordinates
    return 0

# make a publisher that publishes position of bowl.
# subscriber to the camera
# make a subscriber to the topic that tells us that we are in bowl mode

pub = rospy.Publisher('/bowl_cords', Int32MultiArray, queue_size=1)
sub_bool = rospy.Subscriber("/find_bowl_trigger", Bool, bool_callback)
color_sub = rospy.Subscriber("/color/image_raw", Image, color_camera_calback)
depth_sub = rospy.Subscriber("/depth/image_raw", Image, depth_camera_callback)


if __name__ == '__main__':
    rospy.init_node('moveit_node')

    print("This works!")
    

    rospy.spin()