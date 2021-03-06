#! /usr/bin/env python
import rospy 
import rospkg 

import matplotlib.pyplot as plt
import numpy as np

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SetLightProperties
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
from autonomous_eating.msg import face_cords

global face_coordinates
face_coordinates = []

def moveModel(model, position, r, p, y):
    rospy.wait_for_service('/gazebo/set_model_state')
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

def deleteModel(model):
    rospy.wait_for_service("/gazebo/delete_model")  
    try:
        service = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        req = DeleteModel._request_class()
        req.model_name = model
        resp = service(req)
        if resp.success:
            print("Removed human model")
    except rospy.ServiceException, e:
        print("Service call failed: %s" %e)

def spawnUrdfModel(model, pose):
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    try:
        service = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        req = SpawnModel._request_class()
        req.model_name = "human"
        file = open(model, 'r')
        req.model_xml = file.read()
        req.robot_namespace = ""
        req.initial_pose = pose
        req.reference_frame = ""
        resp = service(req)
        if(resp.success):
            print("Succesfully spawned model")
        else:
            print("Failed to print model")
            print(resp.status_message)
    except rospy.ServiceException, e:
        print("Service call failed: %s" %e)

def spawnSdfModel(model, pose):
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    try:
        service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        req = SpawnModel._request_class()
        req.model_name = "human"
        file = open(model, 'r')
        req.model_xml = file.read()
        req.robot_namespace = ""
        req.initial_pose = pose
        req.reference_frame = ""
        resp = service(req)
        if(resp.success):
            print("Succesfully spawned model")
        else:
            print("Failed to spawn model:")
            print(resp.status_message)
    except rospy.ServiceException, e:
        print("Service call failed: %s" %e)

def face_cords_callback(data):
    face_coordinates.append(data)


def runTest(og_pose, lights):
    up_down_range = 130 #75 degrees
    left_right_range = 156 # 90 degrees
    step = 10
    scaling = 100.0
    rad2deg = 180.0/3.14
    sleepTime = 0.2


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
            for lr in range(-left_right_range, left_right_range, step):
                #empty face_coordinates list:
                for a in face_coordinates:
                    face_coordinates.pop()

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
                
                #save amount of detections
                data.append(len(face_coordinates))
            
            dataArray.append(data)

        dataArrayArray.append(dataArray)

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

    print(result)

    #print "DataArray length:",
    #print len(dataArray)

    #print(dataArray)

    #y = [0.1, 0.2, 0.3, 0.4, 0.5]

    #used for plotting
    y = []
    x = []
    for lr in range(-left_right_range, left_right_range, step):
        x.append((lr/scaling)*rad2deg)

    for i in range(-up_down_range, up_down_range, step):
        y.append((i/scaling)*rad2deg)


    print len(x)
    print len(y)
    #setup the 2D grid with Numpy
    x, y = np.meshgrid(x, y)

    #convert intensity (list of lists) to a numpy array for plotting
    intensity = np.array(result)

    #plot as colormesh
    plt.pcolormesh(x, y, intensity, vmax=40)
    #need a colorbar to show the intensity scale
    plt.colorbar() 
    plt.xlabel('Horizontal rotation in degrees')
    plt.ylabel('Vertical tilt in degrees')
    plt.title('Face detection heatmap')
    plt.savefig("/home/ubuntu/Desktop/catkin_ws/src/ae_testing/extra/face_detect_output/20")
    plt.show() 

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

if __name__ == '__main__':
    rospy.init_node('human_pos_node')
    rospy.Subscriber('/face_cords', face_cords, face_cords_callback)
    #deleteModel("human")
    #spawnSdfModel("/home/ubuntu/Desktop/catkin_ws/src/ae_testing/models/1/model.sdf", pose)
    #spawnUrdfModel("/home/ubuntu/Desktop/catkin_ws/src/ae_testing/urdf/1.urdf", pose)

    lights = ['light1', 'light2','light3','light4','light5']

    og_pose = Pose()
    og_pose.position.x = -0.09
    og_pose.position.y = 1.44
    og_pose.position.z = 1.15

    #light_pose = Pose()
    #light_pose.position.x = 0.07
    #light_pose.position.y = -1.54
    #light_pose.position.z = 0.2
    
    #deleteModel("human")
    #spawnSdfModel("/home/ubuntu/Desktop/catkin_ws/src/autonomous_eating/models/human1/model.sdf", og_pose)
    runTest(og_pose, lights)
    #once done, return to original position:

    #setLight(lights[0], 0,0,0)
    #chooseLight(lights, 0)
    moveModel('human', og_pose.position, 0, 0 ,0)
