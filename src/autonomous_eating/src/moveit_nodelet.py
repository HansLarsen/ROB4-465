#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
from tf.transformations import quaternion_from_euler
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

robotName = "j2n6s300"
group_name = "arm"
rootFrame = 'root'
cameraAttachFrame = 'j2n6s300_link_6'
cameraNameFrame = 'realsense'
#x, y, z, roll, yaw, pitch
cameraTransformation = [0, 0, 0.2, 0, 0, 0]

camera_Message = geometry_msgs.msg.Transform()

camera_Message.transform.translation.x = cameraTransformation[0]
camera_Message.transform.translation.y = cameraTransformation[1]
camera_Message.transform.translation.z = cameraTransformation[2]

quad = quaternion_from_euler(cameraTransformation[3], cameraTransformation[4], cameraTransformation[5])

camera_Message.transform.rotation.x = quad[0]
camera_Message.transform.rotation.y = quad[1]
camera_Message.transform.rotation.z = quad[2]
camera_Message.transform.rotation.w = quad[3]

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial')

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                            moveit_msgs.msg.DisplayTrajectory,
                                            queue_size=20)

    rate = rospy.Rate(10.0)

    group = moveit_commander.MoveGroupCommander(group_name)
    robot = moveit_commander.RobotCommander()

    planning_frame = group.get_planning_frame()
    eef_link = group.get_end_effector_link()
    group_names = robot.get_group_names()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    camera_Broadcaster = tf2_ros.TransformBroadcaster()

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