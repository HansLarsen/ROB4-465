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

class MoveitApp():
    def __init__(self):
        self.robotName = "j2n6s300"
        self.group_name = "arm"
        self.rootFrame = 'j2n6s300_link_base'
        self.endEffectFrame = 'j2n6s300_end_effector'
        self.publisher = rospy.Publisher('/Doneski', Bool, queue_size=5)

        rospy.Subscriber('/move_to',
                    String,
                    self.move_to_callback)

        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.robot = moveit_commander.RobotCommander()
        
        self.group.set_pose_reference_frame(self.rootFrame)
        self.group.set_end_effector_link(self.endEffectFrame)

        self.camera_transform = geometry_msgs.msg.Pose()

        self.camera_transform.position.x = -0.421324757876
        self.camera_transform.position.y = -0.146568152514
        self.camera_transform.position.z = 0.282542875551

        self.camera_transform.orientation.x = 0.0187049271885
        self.camera_transform.orientation.y = -0.745113187307
        self.camera_transform.orientation.z = -0.666430957824
        self.camera_transform.orientation.w = -0.0180621774537

    def move_to_callback(self, data):
        self.group.stop()
        self.group.clear_pose_targets()
        goal_transform = []
        if (data.data=="stop"):
            self.group.stop()
            self.group.set_pose_target(self.camera_transform)
            self.group.go(wait=True)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_node')

    myMoveitApp = MoveitApp()
    rospy.spin()