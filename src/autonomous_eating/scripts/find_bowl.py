#! /usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import Header
from std_msgs.msg import Int32MultiArray, Bool
from sensor_msgs.msg import Image

from cv_bridge import CvBridge


def bool_callback(data):
    return data


def color_camera_calback(data):
    try:
        rgb_image = CvBridge().imgmsg_to_cv2(image_message, desired_encoding="rgb8")

    except CvBridgeError as e:
        print(e)

    cv2.imshow("rgb", rgb_image)


def depth_camera_callback(data):
    return 0

def find_biggest_contour(image):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
    biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

    return biggest_contour

def bowl_finder(image):
    #resize so all picture are same size
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    max_dimension = max(image.shape)
    scale = 700/max_dimension
    image = cv2.resize(image, None, fx = scale, fy = scale)

    #blur

    image_blur = cv2.GaussianBlur(image,(7,7), 0)
    hsv_image_blur = cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)

    #threshold
    min_red = np.array([0, 170, 40])
    max_red = np.array([20, 256,256])

    #color threshold
    filter1 = cv2.inRange(hsv_image_blur, min_red, max_red)

    #color threshold 2
    min_intens_red = np.array([170, 170, 40])
    max_intens_red = np.array([180, 256, 256])
    filter2 = cv2.inRange(hsv_image_blur, min_intens_red, max_intens_red)

    filter = filter1 + filter2

    #segementation
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
    morph_close = cv2.morphologyEx(filter, cv2.MORPH_CLOSE, kernel )
    morph_open = cv2.morphologyEx(morph_close, cv2.MORPH_OPEN, kernel)

    #find biggest contour (subject to change)
    biggest_contour = find_biggest_contour(morph_open)

    #find center

    M = cv2.moments(biggest_contour)

    center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]) )
    #cx = int(M["m10"]/M["m00"])
    #cy = int(M["m01"]/M["m00"])

    return center


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
