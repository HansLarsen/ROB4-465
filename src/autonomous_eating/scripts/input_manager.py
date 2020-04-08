#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

numb_but = 4
input_commands = [0 for x in range(3+numb_but)]
msgs = Int32MultiArray()

def joy_callback(data):
    input_commands[0] = int(data.axes[0]*100)
    input_commands[1] = int(data.axes[1]*100)
    input_commands[2] = int(data.axes[4]*100)

    for i in range(numb_but):
        input_commands[i+3] = int(data.buttons[i])

    msgs.data = input_commands
    pub.publish(msgs)

def au_position_callback(data):
    input_commands[0] = int(data.x)
    input_commands[1] = int(data.y)
    input_commands[2] = int(data.theta)

    msgs.data = input_commands
    pub.publish(msgs)

def pushed_buttons_callback(data):
    rospy.loginfo("pushed_buttons_callback activated")
    # for i in range(numb_but):
    #     input_commands[i+3] = 0

    # if data.data == "first_button":
    #     input_commands[0] = 1
    # elif data.data == "second_button":
    #     input_commands[1] = 1
    # elif data.data == "third_button":
    #     input_commands[2] = 1 
    # elif data.data == "fourth_button":
    #     input_commands[3] = 1      

    # msgs.data = input_commands
    # pub.publish(msgs)

if __name__ == '__main__':
    rospy.init_node('input_manager', anonymous=True)

    use_joy = rospy.get_param('joy')


    pub = rospy.Publisher('input_commands', Int32MultiArray, queue_size=10)

    if use_joy:
        rospy.loginfo("Use Joy: True")
        rospy.Subscriber("joy", Joy, joy_callback)
        
    
    if not use_joy:
        rospy.loginfo("Use Joy: False")
        rospy.Subscriber("/itci/au_position", Pose2D, au_position_callback)
        rospy.Subscriber("/itci/pushed_buttons", String, pushed_buttons_callback)

    while not rospy.is_shutdown():
        rospy.spin()
    