#! /usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf2_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray, Int32
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list
from enum import Enum

class MoveitApp():
    def __init__(self, master):
        self.robotName = "j2n6s300"
        self.group_name = "arm"
        self.rootFrame = 'root'
        self.cameraAttachFrame = 'j2n6s300_link_6'
        self.cameraNameFrame = 'realsense'

        camera_transform = geometry_msgs.msg.Transform()
        bowl_transform = geometry_msgs.msg.Transform()

        #x, y, z, roll, yaw, pitch
        self.cameraTransformation = [0, 0.2, 0, 0, 0, 0]

        self.camera_Message = geometry_msgs.msg.TransformStamped()

        self.camera_Message.header.frame_id = self.cameraAttachFrame
        self.camera_Message.child_frame_id = self.cameraNameFrame

        self.camera_Message.transform.translation.x = self.cameraTransformation[0]
        self.camera_Message.transform.translation.y = self.cameraTransformation[1]
        self.camera_Message.transform.translation.z = self.cameraTransformation[2]

        self.quad = quaternion_from_euler(self.cameraTransformation[3], self.cameraTransformation[4], self.cameraTransformation[5])

        self.camera_Message.transform.rotation.x = self.quad[0]
        self.camera_Message.transform.rotation.y = self.quad[1]
        self.camera_Message.transform.rotation.z = self.quad[2]
        self.camera_Message.transform.rotation.w = self.quad[3]

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                            moveit_msgs.msg.DisplayTrajectory)

        self.status_publisher = rospy.Publisher('/status_jaco',
                                            Int32MultiArray, queue_size=20)

        rospy.Subscriber('/tf',
                    tf2_ros.TFMessage,
                    self.pose_callback,
                    robotName)

        rospy.Subscriber('/move_to',
                    String,
                    self.move_to_callback,
                    robotName)

        rospy.Subscriber('/move_xy',
                    Int32MultiArray,
                    self.move_xy_callback,
                    robotName)

        rospy.Subscriber('/move_capture',
                    Int32,
                    self.move_capture,
                    robotName)

        rospy.Subscriber('/bowl_cords',
                    Int32MultiArray,
                    self.bowl_cords_callback,
                    robotName)

        rospy.Subscriber('/face_cords',
                    Int32MultiArray,
                    self.face_cords_callback,
                    robotName)

        self.rate = rospy.Rate(10.0)

        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.robot = moveit_commander.RobotCommander()

        self.planning_frame = group.get_planning_frame()
        self.eef_link = group.get_end_effector_link()
        self.group_names = robot.get_group_names()

        tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(tfBuffer)
        self.camera_Broadcaster = tf2_ros.TransformBroadcaster()


    def pose_callback(pose, another):
        self.camera_Message.header.stamp = rospy.Time.now()
        self.camera_Broadcaster.sendTransform(camera_Message)

    def move_to_callback(data):
        goal_transform = []
        if (data=="mouth"):
            #TODO: move to the positon
        else if (data == "bowl_search_pos"):
            continue
        else if (data == "scoop_bowl"):
            continue
        else if (data == "face_seach_pos"):
            

    def move_xy_callback(data):
        return

    def bowl_cords_callback(data):
        return

    def face_cords_callback(data):
        return

    def move_capture(data):
        try:
            #First frame, end frame
            trans = tfBuffer.lookup_transform(rootFrame, cameraAttachFrame, rospy.Time(0))

            if (data == 0):
                camera_transform = trans
                rospy.loginfo("Saved face capture position")
            else if (data == 1):
                bowl_transform = trans
                rospy.loginfo("Saved bowl capature position")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to save position")

        return

    def transmit_moving(moving_boolean):
        msg = Int32MultiArray()
        #TODO: get jaco conencted status
        msg.data.append(0)
        msg.data.append(moving_boolean)
        self.status_publisher.publish(msg)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_node')



    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.z = 0.2
    wpose.position.y = 0.6
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = 0.6
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y = 0.5
    waypoints.append(copy.deepcopy(wpose))

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
        #    trans = tfBuffer.lookup_transform(rootFrame, cameraAttachFrame, rospy.Time(0))
        #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #    rate.sleep()
        #    rospy.logerr("Failed")
        #    continue