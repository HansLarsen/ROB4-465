#! /usr/bin/env python
import rospy 
import rospkg 

import matplotlib.pyplot as plt
import numpy as np

from std_msgs.msg import Int32
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetLightProperties
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from autonomous_eating.msg import face_cords
import moveit_commander
import moveit_msgs.msg
from gazebo_msgs.srv import GetModelState
from scipy import stats
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
import sys
import copy
import numpy as np
import time
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list
from enum import Enum






global face_coordinates
global images_processed
images_processed = -1
face_coordinates = []

def moveModel(model, position, r, p, y):
    try:
        service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        twist = Twist()
        pose = Pose()
        pose.position = position
        #rpy to quaternion:
        quat = quaternion_from_euler(r, p, y)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        req = SetModelState()
        req.model_name = model
        req.pose = pose
        req.twist = twist
        req.reference_frame = ""

        resp = service(req)

    except rospy.ServiceException, e:
        print("Service call failed: %s" %e)


def face_cords_callback(data):
    face_coordinates.append(data)

def img_proc_callback(data):
    global images_processed
    images_processed = data.data

def setLight(light, r,g,b):
    try:
        service = rospy.ServiceProxy('/gazebo/set_light_properties', SetLightProperties)

        req = SetLightProperties._request_class()
        req.light_name = light
        req.diffuse.r = r
        req.diffuse.g = g
        req.diffuse.b = b
        req.diffuse.a = 0
        req.attenuation_constant = 1
        req.attenuation_linear = 1
        req.attenuation_quadratic = 0

        resp = service(req)

    except rospy.ServiceException, e:
        print("Service call failed: %s" %e)

def chooseLight(lights, light):
    for a in range(0,len(lights)):
        setLight(lights[a], 0,0,0)

    setLight(lights[light], 1,1,1)

def runTest(og_pose, lights):
    ## face detection
    up_down_range = 25 #15 degrees
    left_right_range = 80 # 45 degrees
    step = 5
    scaling = 100.0
    rad2deg = 180.0/3.14
    sleepTime = 1

    ## landmark detection
    rootFrame = 'world'
    cameraNameFrame = 'color_corrected_frame'
    tfBuffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tfBuffer)
    x = []
    y = []
    z = []

    global images_processed
    #wait until images_processed topic has been received once:
    while(images_processed < 0):
        rospy.sleep(1)

    start_images_processed = images_processed
    faces_detected = 0

    #loop through lighting settings:
    dataArrayArray = []
    for i in range(0,len(lights)):
        chooseLight(lights, i)
        print "lighting setting ",
        print i

        dataArray = []
        for up in range(-up_down_range, up_down_range, step):
            #theta for up/down angle
            theta_ud = up/scaling
            data = []
            #save y for graphing
            for lr in range(-left_right_range, left_right_range, step*2):
                #empty face_coordinates list:
                face_coordinates[:] = []
                #for a in face_coordinates:
                #    face_coordinates.pop()

                #theta for left/right angle
                theta_lr = lr/scaling
                #move the model
                moveModel('human', og_pose.position, theta_ud, 0.0, theta_lr)

                #give time to detect face
                rospy.sleep(sleepTime)
                
                #once we have slept, remove noice            
                for point in face_coordinates:
                    if point.z_p1 > 1000:
                        face_coordinates.pop(face_coordinates.index(point))
                        #print("removed some noise")

                faces_detected = faces_detected + len(face_coordinates)
                
                #save amount of detections (face detections)
                data.append(len(face_coordinates))

                #calculate position relative to world of face_cords
                for faces in face_coordinates:
                    target_pose = PoseStamped()
                    target_pose.pose.position.x = faces.x_p2/1000.0
                    target_pose.pose.position.y = faces.y_p2/1000.0
                    target_pose.pose.position.z = faces.z_p2/1000.0
                    target_pose.header.frame_id = cameraNameFrame
                    target_pose.header.stamp = rospy.Time.now()

                    try:
                        camera_transforms = tfBuffer.lookup_transform(rootFrame, cameraNameFrame, rospy.Time(0))

                        target_transformed_pose = tf2_geometry_msgs.do_transform_pose(target_pose, camera_transforms)

                        x.append(target_transformed_pose.pose.position.x)
                        y.append(target_transformed_pose.pose.position.y)
                        z.append(target_transformed_pose.pose.position.z)

                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rospy.spin()
                        rospy.loginfo("Failed lookup")
                        return

                
            
            dataArray.append(data)

        dataArrayArray.append(dataArray)

    end_images_processed = images_processed
    #calculate final detection rate, on a per frame basis:
    total_images_processed = end_images_processed - start_images_processed
    detection_rate = float(faces_detected) / float(total_images_processed)
    detection_rate = round(detection_rate * 100.0, 2)
    print "",
    print "images processed: ",
    print total_images_processed,
    print ", detected faces: ",
    print faces_detected,
    print ", face detection rate: ",
    print detection_rate,
    print "%"


    ## landmark detection results
    try:
        service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        req = GetModelState._request_class()
        req.model_name = 'human'
        req.relative_entity_name = ''

        resp = service(req)
    except rospy.ServiceException, e:
        print("Service call failed: %s" %e)

    #print "human pose.position: "
    #print resp.pose.position
    #print ''

    xArray = (np.asarray(x)-resp.pose.position.x)*1000.0
    yArray = (np.asarray(y)-resp.pose.position.y)*1000.0
    zArray = (np.asarray(z)-resp.pose.position.z)*1000.0

    meanX, sigmaX = np.mean(xArray), np.std(xArray)
    conf_intX = stats.norm.interval(0.95, loc=meanX, scale=sigmaX/np.sqrt(len(xArray)))
    conf_intX = conf_intX[0] - conf_intX[1]
    meanY, sigmaY = np.mean(yArray), np.std(yArray)
    conf_intY = stats.norm.interval(0.95, loc=meanY, scale=sigmaY/np.sqrt(len(yArray)))
    conf_intY = conf_intY[0] - conf_intY[1]
    meanZ, sigmaZ = np.mean(zArray), np.std(zArray)
    conf_intZ = stats.norm.interval(0.95, loc=meanZ, scale=sigmaZ/np.sqrt(len(zArray)))
    conf_intZ = conf_intZ[0] - conf_intZ[1]

    print "X, ",
    print round(meanX, 1),
    print ',',
    print round(abs(conf_intX), 1),
    print ',',
    print round(sigmaX, 1),

    print ",Y, ",
    print round(meanY, 1),
    print ',',
    print round(abs(conf_intY), 1),
    print ',',
    print round(sigmaY, 1),
    
    print ",Z, ",
    print round(meanZ, 1),
    print ',',
    print round(abs(conf_intZ), 1),
    print ',',
    print round(sigmaZ, 1),
    print ',',

    print "detection rate,",
    print detection_rate,
    print ','




    ## face detection results
    result = dataArrayArray[0]
    for i in range(len(result)):
        for j in range(len(result[0])):
            result[i][j] = 0

    for i in range(len(result)):
        for j in range(len(result[0])):
            a = 0
            for l in range(len(dataArrayArray)):
                a = a + dataArrayArray[l][i][j]
            result[i][j] = a

    #print(result)

    #used for plotting
    y = []
    x = []
    for lr in range(-left_right_range, left_right_range, step*2):
        x.append((lr/scaling)*rad2deg)

    for i in range(-up_down_range, up_down_range, step):
        y.append((i/scaling)*rad2deg)

    #setup the 2D grid with Numpy
    x, y = np.meshgrid(x, y)

    #convert intensity (list of lists) to a numpy array for plotting
    intensity = np.array(result)

    #plot as colormesh
    plt.pcolormesh(x, y, intensity, vmax=18)
    #need a colorbar to show the intensity scale
    plt.colorbar() 
    plt.xlabel('Horizontal rotation in degrees')
    plt.ylabel('Vertical tilt in degrees')
    plt.title('Face detection heatmap')
    plt.savefig("/home/ubuntu/Desktop/catkin_ws/src/ae_testing/extra/face_detect_output/20")
    plt.show() 



if __name__ == '__main__':
    rospy.init_node('face_detect_test_node')
    rospy.Subscriber('/face_cords', face_cords, face_cords_callback)
    rospy.Subscriber('/images_processed', Int32, img_proc_callback)
    

    lights = ['light1', 'light2','light3','light4','light5', 'light6']
    lights = ['light1','light2', 'light3']

    og_pose = Pose()
    og_pose.position.x = -0.09
    og_pose.position.y = 1.44
    og_pose.position.z = 1.15

    #light_pose = Pose()
    #light_pose.position.x = 0.07
    #light_pose.position.y = -1.54
    #light_pose.position.z = 0.2
    
    runTest(og_pose, lights)
    #once done, return to original position:
    moveModel('human', og_pose.position, 0.0, 0.0, 0.0)