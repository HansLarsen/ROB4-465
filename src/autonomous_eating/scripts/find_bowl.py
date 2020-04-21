#! /usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import Header
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Bool
from sensor_msgs.msg import Image
from autonomous_eating.srv import *
from cv_bridge import CvBridge
import image_geometry

def deproject_func(x,y,z):
    rospy.wait_for_service()
    resp = deprojectResponse("deproject_pixel_to_world")
    try:
        service = rospy.ServiceProxy("deproject_pixel_to_world", deproject)
        req = deprojectRequest()
        req.x = x
        req.y = y
        req.z = z
        resp = service(req)
    except rospy.ServiceException, e:
        print("Service call failed: %s" %e)

    return resp

def bool_callback(data): #finds out if we should find bowl
    global search
    search = data.data
    
def color_camera_calback(data): #converts the given image from camera into a CV2 image
    global bgr_image
    try:
        bgr_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    

def depth_camera_callback(data): #converts the given image from camera into a CV2 image
    global depth_image
    try:
        depth_image = CvBridge().imgmsg_to_cv2(data,"32FC1")
    except CvBridgeError as e:
        print(e)

def cordinatecallback(data):  #recieves 3D cordinates and publishes them on /bowl_cords
    global object_out_of_range
    if data.data[2] !=0:
        msg = Float32MultiArray()
        array = [data.data[0], data.data[1], data.data[2]]
        msg.data = array
        pub_bowl.publish(msg)
        object_out_of_range = False
    else: 
        #print "object is out of range"
        object_out_of_range = True
    


def find_biggest_contour(image): #figures out which of the found contours is the biggest
    image = image.copy()
    im2, contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
    biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

    return biggest_contour

def draw_square(image, biggest_contour): #draws circle on picture
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #radius = 200
    color = (0, 0, 0)
    thickness = 2
    #cv2.circle(image, center, radius, color, thickness)
    #image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    rect = cv2.boundingRect(biggest_contour)
    cv2.rectangle(image,(rect[0],rect[1]),(rect[0]+rect[2],rect[1]+rect[3]),color,thickness)
    return image

def bowl_finder(image): #finds the center coordinats of the bowl
    #resize so all picture are same size
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #max_dimension = max(image.shape)
    #scale = 700/max_dimension
    #image = cv2.resize(image, None, fx = scale, fy = scale)

    #blur

    image_blur = cv2.GaussianBlur(image,(7,7), 0)
    hsv_image_blur = cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)

    #threshold
    min_red = np.array([0, 170, 40])
    max_red = np.array([5, 256,256])

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
    global biggest_contour
    biggest_contour = find_biggest_contour(morph_open)

    #find center

    M = cv2.moments(biggest_contour)

    center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]) )
    #cx = int(M["m10"]/M["m00"])
    #cy = int(M["m01"]/M["m00"])
    return center 

#publishers
pub_img = rospy.Publisher('/gui_figure', Image, queue_size=1)
pub_bowl = rospy.Publisher('/bowl_cords', Float32MultiArray, queue_size=1)
pub_pixel = rospy.Publisher('/pixel_Cords', Float32MultiArray, queue_size=1)
#subscribers
sub_bool = rospy.Subscriber("/find_bowl_trigger", Bool, bool_callback)
color_sub = rospy.Subscriber("/r200/camera/color/image_raw", Image, color_camera_calback)
depth_sub = rospy.Subscriber("/r200/camera/depth/image_raw", Image, depth_camera_callback)
sub_cord3d = rospy.Subscriber("/3D_cordinates", Float32MultiArray, cordinatecallback)

debug = False   
search = True
object_out_of_range = False 
found_data = False
UsingSimulatedCam = True

data_color = None
data_depth = None



if __name__ == '__main__':
    rospy.init_node('find_bowl')

    while not found_data: #checks if camera is publishing
        try:
            data_color = rospy.wait_for_message("/r200/camera/color/image_raw", Image, timeout= 5)
            data_depth = rospy.wait_for_message("/r200/camera/depth/image_raw", Image, timeout= 5)
            if data_color is not None and data_depth is not None:
                found_data = True
        except:
            if debug == True:
                print("excepting")
            if data_color is None:
                rospy.loginfo("Did not find color camera")
            if data_depth is None:
                rospy.loginfo("Did not find depth camera")
            
    while not rospy.is_shutdown():
        
        ran = False                     #this is the bool that tells us if a red bowl is found

        if(search == True):             #checks if we should find the bowl
            if debug == True:
                print "searching! ",
           
            try: 
                global c
                c = bowl_finder(bgr_image)          #tries to find bowl and gets center coordinates
                d = depth_image[c]                      #gets depth
        
                resp = deproject_func(c[0],c[1], d)     #sends center and depth to get real coordinates

                if debug == True:
                    print "found it!"
                ran = True                              #it has found the bowl and published it

            except:
                if debug == True:
                    print "did not find bowl"
                
            if ran == True and object_out_of_range == False : #if bowl was found, it publishes the camerafeed with marking of object                              
                global biggest_contour
                image_to_publish = draw_square(bgr_image, biggest_contour)
                image_msg = CvBridge().cv2_to_imgmsg(image_to_publish, "rgb8")
                pub_img.publish(image_msg)

                data_to_send = Float32MultiArray() 
                data_to_send.data = [resp.x, resp.y, resp.z]
                pub_bowl.publish(data_to_send)
                
            else:
                if debug:
                    print("bowl not found")
                image_p = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)        #if it was not found publish regular camera feed
                image_to_publish = CvBridge().cv2_to_imgmsg(image_p, "rgb8")
                pub_img.publish(image_to_publish)
            
        else:
            if debug == True:                                   #if we are not in bowl searhing position it does nothing
                print "not searching"        

        
        
        
