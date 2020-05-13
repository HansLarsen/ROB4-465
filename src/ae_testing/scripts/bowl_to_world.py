#! /usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
import sys
import copy
import numpy as np
import time
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray, Int32, Float32MultiArray, Bool
from autonomous_eating.msg import face_cords
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list
from enum import Enum

got_bowl = False

class MoveitApp():
    def __init__(self):
        self.rootFrame = 'world'
        self.cameraNameFrame = 'color_corrected_frame'

        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.Subscriber('/bowl_cords',
                    Float32MultiArray,
                    self.bowl_callback)

        self.counter = 0

    def bowl_callback(self, data):
        if self.counter < 30:
            self.counter = self.counter + 1
            return

        self.counter = 0

        target_pose = geometry_msgs.msg.PoseStamped()

        target_pose.pose.position.x = data.data[0]/1000.0
        target_pose.pose.position.y = data.data[1]/1000.0
        target_pose.pose.position.z = data.data[2]/1000.0
        target_pose.header.frame_id = self.cameraNameFrame
        target_pose.header.stamp = rospy.Time.now()

        try:
            camera_transforms = self.tfBuffer.lookup_transform(self.rootFrame, self.cameraNameFrame, rospy.Time(0))

            target_transformed_pose = tf2_geometry_msgs.do_transform_pose(target_pose, camera_transforms)

            rospy.loginfo(target_transformed_pose)

            diffX =round(-realcoordsX+(target_transformed_pose.pose.position.x*1000.0),3)
            diffY =round(-realcoordsY+(target_transformed_pose.pose.position.y*1000.0),3)
            diffZ =round(-realcoordsZ+(target_transformed_pose.pose.position.z*1000.0),3)

            
            print("Diff x: ",diffX)
            print("Diff y: ",diffY)
            print("Diff z: ",diffZ)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.spin()
            rospy.loginfo("Failed lookup")
            return
        

if __name__ == '__main__':
    if not got_bowl:
        try:
            realcoordsX=float(input('Enter real bowl x coordinate in mm: '))
            realcoordsY=float(input('Enter real bowl y coordinate in mm: '))
            realcoordsZ=float(input('Enter real bowl z coordinate in mm: '))

            got_bowl = True
        except ValueError:
            print("not a number")
        
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_node')

    myMoveitApp = MoveitApp()
    rospy.spin()