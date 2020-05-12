#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from autonomous_eating.msg import command_msg
from std_msgs.msg import Bool, Empty, String

msg = command_msg()
buttonPressed = 0
buttonTime = 0.0

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
    global buttonTime
    global buttonPressed
    msg.x = int(data.x)
    msg.y = int(data.y)
    msg.z = int(data.theta)

    msg.button1 = 0
    msg.button2 = 0
    msg.button3 = 0
    msg.mode_select = 0

    if msg.x < 3.5 and msg.y > 27:
        if buttonPressed == 1:
            if rospy.get_time() - buttonTime > 1:
                msg.button1 = 1
        else:
            buttonPressed = 1
            buttonTime = rospy.get_time()
            
    elif msg.x > 3.5 and msg.x < 6.5 and msg.y > 27:
        if buttonPressed == 2:
            if rospy.get_time() - buttonTime > 1:
                msg.button2 = 1
        else:
            buttonPressed = 2
            buttonTime = rospy.get_time()

    elif msg.x > 6.5 and msg.y > 27:
        if buttonPressed == 3:
            if rospy.get_time() - buttonTime > 1:
                msg.button3 = 1
        else:
            buttonPressed = 3
            buttonTime = rospy.get_time()

    elif msg.y < 27 and msg.y > 20:
        if buttonPressed == 4:
            if rospy.get_time() - buttonTime > 1:
                msg.mode_select = 1
        else:
            buttonPressed = 4
            buttonTime = rospy.get_time()
    else:
        buttonPressed = 0
    if msg.y > 15:
        msg.x = 0
        msg.y = 0
    else:
        msg.x = (msg.x-5)*20
        msg.y = (msg.y-5)*20

    pub.publish(msg)

#def pushed_buttons_callback(data):
 #   return
    #rospy.loginfo("pushed_buttons_callback activated")
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
        global buttonTime
        global buttonPressed
        buttonPressed = 0
        buttonTime = 0.0
        rospy.loginfo("Using ITCI")
        rospy.Subscriber("/itci/au_position", Pose2D, au_position_callback)
        in_handPub = rospy.Publisher('/itci/in_hand', Bool, queue_size=1)
        connectPub = rospy.Publisher('/itci/connect_itci', Bool, queue_size=1)
        #rospy.Subscriber("/itci/pushed_buttons", String, pushed_buttons_callback)
        rospy.sleep(1)
        in_handPub.publish(True)
        connectPub.publish(True)
    else:
        rospy.loginfo("Using Joy")
        rospy.Subscriber("joy", Joy, joy_callback)

    while not rospy.is_shutdown():
        rospy.spin()

        


    