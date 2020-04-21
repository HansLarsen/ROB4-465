#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from autonomous_eating.msg import command_msg

msg = command_msg()

def joy_callback(data):
    msg.x = int(data.axes[0]*100)
    msg.y = int(data.axes[1]*100)
    msg.z = int(data.axes[4]*100)

    msg.button1 = int(data.buttons[0])
    msg.button2 = int(data.buttons[1])
    msg.button3 = int(data.buttons[2])
    msg.mode_select = int(data.buttons[3])

    pub.publish(msg)

def au_position_callback(data):
    msg.x = int(data.x)
    msg.y = int(data.y)
    msg.z = int(data.theta)

    pub.publish(msg)

def pushed_buttons_callback(data):
    rospy.loginfo("pushed_buttons_callback activated")
    # if data.data == "first_button":
    #     msg.button1 = 1
    # elif data.data == "second_button":
    #     msg.button2 = 1
    # elif data.data == "third_button":
    #     msg.button3 = 1 
    # elif data.data == "fourth_button":
    #     msg.mode_select = 1      

    # pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('input_manager', anonymous=True)

    pub = rospy.Publisher('input_commands', command_msg, queue_size=10)

    if rospy.get_param('/itci', False):
        rospy.loginfo("Use ITCI: True")
        rospy.Subscriber("/itci/au_position", Pose2D, au_position_callback)
        rospy.Subscriber("/itci/pushed_buttons", String, pushed_buttons_callback)
  
    else:
        rospy.loginfo("Use ITCI: False")
        rospy.Subscriber("joy", Joy, joy_callback)

    while not rospy.is_shutdown():
        rospy.spin()
    