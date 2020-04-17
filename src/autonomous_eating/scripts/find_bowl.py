#! /usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import Header
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import image_geometry

search = False


def bool_callback(data):
    global search
    search = data.data
    


def color_camera_calback(data):
    global bgr_image
    try:
        bgr_image = CvBridge().imgmsg_to_cv2(data, "bgr8")


    except CvBridgeError as e:
        print(e)
    

def depth_camera_callback(data):
    global depth_image
    try:
        depth_image = CvBridge().imgmsg_to_cv2(data,"32FC1")
       
       
    except CvBridgeError as e:
        print(e)

def cordinatecallback(data):
    return (data.data)



def find_biggest_contour(image):
    image = image.copy()
    im2, contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
    biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

    return biggest_contour

#def draw_center(image, center):

    #radius = 200
    #color = (0, 0, 0)
    #thickness = 2
    #cv2.circle(image, center, radius, color, thickness)

def bowl_finder(image):
    
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
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

    biggest_contour = find_biggest_contour(morph_open)

    #find center

    M = cv2.moments(biggest_contour)

    center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]) )
    #cx = int(M["m10"]/M["m00"])
    #cy = int(M["m01"]/M["m00"])
    #draw_center(image, center)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return center, image

pubimg = rospy.Publisher('/gui_figure', Image, queue_size=1)
pub = rospy.Publisher('/bowl_cords', Int32MultiArray, queue_size=1)
pub_pixel = rospy.Publisher('/pixel_Cords', Float32MultiArray, queue_size=1)
sub_bool = rospy.Subscriber("/find_bowl_trigger", Bool, bool_callback)
color_sub = rospy.Subscriber("/camera/color/image_raw", Image, color_camera_calback)
depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, depth_camera_callback)
sub_cord3d = rospy.Subscriber("/3D_cordinates", Float32MultiArray, cordinatecallback)





if __name__ == '__main__':
    
    rospy.init_node('find_bowl')
    
    if(search == False):
        print "searching!"
        #bgr2_image = cv2.imread("/home/ubuntu/Desktop/mine//Bowl_Movements_138.jpg")
        try: 
            c, i = bowl_finder(bgr_image)
            d = depth_image[c]
            #print(depth_image[c])
            #print(c)

            #cv2.imshow("pic", bgr_image)
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()
            image_send = Image()
            data_to_send = Float32MultiArray() 
            image_send.data = i
            array = [c[0],c[1], d]
            data_to_send.data = array 
            pub_pixel.publish(data_to_send)
            pubimg.publish(image_send)
        
        except:
            print("error maybe no bowl")
    
        
    rospy.spin()
