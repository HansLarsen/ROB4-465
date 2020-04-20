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
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray, Int32, Float32MultiArray
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
        self.cameraNameFrame = 'r200_realsense'
        self.movement_factor = 0.1

        self.camera_transform = geometry_msgs.msg.Pose()
        self.bowl_transform = geometry_msgs.msg.Pose()

        self.camera_transform.position.x = -0.421324757876
        self.camera_transform.position.y = -0.146568152514
        self.camera_transform.position.z = 0.282542875551

        self.camera_transform.orientation.x = 0.0187049271885
        self.camera_transform.orientation.y = -0.745113187307
        self.camera_transform.orientation.z = -0.666430957824
        self.camera_transform.orientation.w = -0.0180621774537

        self.bowl_transform.position.x = 0.172054552405
        self.bowl_transform.position.y = -0.319987002705
        self.bowl_transform.position.z = 0.386660997851

        self.bowl_transform.orientation.x = -0.622202411237
        self.bowl_transform.orientation.y = 0.678180666678
        self.bowl_transform.orientation.z = 0.313715889391
        self.bowl_transform.orientation.w = -0.23348979322


        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                            moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        self.status_publisher = rospy.Publisher('/status_jaco',
                                            Int32MultiArray, queue_size=20)

        self.rate = rospy.Rate(10.0)

        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.robot = moveit_commander.RobotCommander()

        self.planning_frame = self.group.get_planning_frame()
        self.eef_link = self.group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.Subscriber('/move_to',
                    String,
                    self.move_to_callback,
                    self.robotName)

        rospy.Subscriber('/move_xy',
                    Int32MultiArray,
                    self.move_xy_callback,
                    self.robotName)

        rospy.Subscriber('/move_capture',
                    Int32,
                    self.move_capture,
                    self.robotName)

        rospy.Subscriber('/bowl_cords',
                    Float32MultiArray,
                    self.bowl_cords_callback,
                    self.robotName)

        rospy.Subscriber('/face_cords',
                    face_cords,
                    self.face_cords_callback,
                    self.robotName)

        self.group.set_pose_reference_frame(self.rootFrame)
        self.group.set_end_effector_link(self.endEffectFrame)

        self.face_cords = face_cords()
        self.bowl_cords = [0, 0, 0]

        rospy.loginfo("============ Reference frame: ")
        rospy.loginfo(self.group.get_pose_reference_frame())
        rospy.loginfo(self.group.get_end_effector_link())

    def move_to_callback(self, data, topic):
        self.group.stop()
        self.group.clear_pose_targets()
        goal_transform = []
        if (data.data=="mouth"):
            self.goto_pose_camera(self.face_cords.x_p1, self.face_cords.y_p1, self.face_cords.z_p1)
        elif (data.data=="mouth2"):
            self.goto_pose_camera(self.face_cords.x_p2, self.face_cords.y_p2, self.face_cords.z_p2)
        elif (data.data == "bowl_search_pos"):
            self.group.set_pose_target(self.bowl_transform)
            self.transmit_moving(True)
            self.group.go(wait=True)
            self.transmit_moving(False)
        elif (data.data == "scoop_bowl"):
            self.goto_pose_camera(self.bowl_cords[0], self.bowl_cords[1], self.bowl_cords[2])
        elif (data.data == "face_search_pos"):
            self.group.set_pose_target(self.camera_transform)
            self.transmit_moving(True)
            self.group.go(wait=True)
            self.transmit_moving(False)

    def goto_pose_camera(self, x, y, z, cameraTransformTrue=True):
        target_pose = geometry_msgs.msg.Pose()

        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        try:
            target_transformed_pose = geometry_msgs.msg.Pose()
            if (cameraTransformTrue == True):
                self.camera_transform = self.tfBuffer.lookup_transform(self.rootFrame, self.cameraNameFrame, rospy.Time(0))
                target_pose.orientation = self.camera_transform.transform.rotation
                target_transformed_pose = tf2_geometry_msgs.do_transform_pose(target_pose, self.camera_transform)
            else:
                self.end_transform = self.tfBuffer.lookup_transform(self.rootFrame, self.endEffectFrame, rospy.Time(0))
                target_pose.orientation = self.end_transform.transform.rotation
                target_transformed_pose = tf2_geometry_msgs.do_transform_pose(target_pose, self.end_transform)
            

            self.group.set_pose_target(target_transformed_pose)
            self.transmit_moving(True)
            self.group.go(wait=True)
            self.transmit_moving(False)
            return True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to lookup transform")
            return False

    def move_xy_callback(self, data, topic):
        newSearchPos = copy.copy(self.bowl_transform)
        newSearchPos.position.x = self.bowl_transform.position.x + float(data.data[0]) * self.movement_factor
        newSearchPos.position.y = self.bowl_transform.position.y + float(data.data[1]) * self.movement_factor
        self.group.set_pose_target(newSearchPos)
        self.transmit_moving(True)
        self.group.go(wait=True)
        self.transmit_moving(False)

    def bowl_cords_callback(self, data, topic):
        self.bowl_cords = np.asarray(data.data)

    def face_cords_callback(self, data, topic):
        self.face_cords = data.data

    def move_capture(self, data, topic):
        try:
            #First frame, end frame
            trans = self.tfBuffer.lookup_transform(self.rootFrame, self.endEffectFrame, rospy.Time(0))

            if (data.data == 0):
                self.camera_transform.position = trans.transform.translation
                self.camera_transform.orientation = trans.transform.rotation
                rospy.loginfo("Saved face capture position")

                print(self.camera_transform.position.x)
                print(self.camera_transform.position.y)
                print(self.camera_transform.position.z)

                print(self.camera_transform.orientation.x)
                print(self.camera_transform.orientation.y)
                print(self.camera_transform.orientation.z)
                print(self.camera_transform.orientation.w)
            elif (data.data == 1):
                self.bowl_transform.position = trans.transform.translation
                self.bowl_transform.orientation = trans.transform.rotation
                rospy.loginfo("Saved bowl capature position")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to save position")

        return

    def transmit_moving(self, moving_boolean):
        msg = Int32MultiArray()
        #TODO: get jaco conencted status
        msg.data.append(0)
        msg.data.append(moving_boolean)
        self.status_publisher.publish(msg)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_node')

    myMoveitApp = MoveitApp()

    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""

    #plan = group.go(wait=True)
    # group.stop()
    # group.clear_pose_targets()

    # waypoints = []

    # wpose = group.get_current_pose().pose
    # wpose.position.z = 0.2
    # wpose.position.y = 0.6
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.x = 0.6
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y = 0.5
    # waypoints.append(copy.deepcopy(wpose))

    #(plan, fraction) = group.compute_cartesian_path(
    #                                waypoints,   # waypoints to follow
    #                                0.01,        # eef_step
    #                                0.0)         # jump_threshold

    #display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #display_trajectory.trajectory_start = robot.get_current_state()
    #display_trajectory.trajectory.append(plan)
    # Publish
    #display_trajectory_publisher.publish(display_trajectory)

    #group.execute(plan, wait=True)

    while not rospy.is_shutdown():
        rospy.spin()
        #try:
            #First frame, end frame
        #    trans = tfBuffer.lookup_transform(rootFrame, endEffectFrame, rospy.Time(0))
        #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #    rate.sleep()
        #    rospy.logerr("Failed")
        #    continue