#! /usr/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
import matplotlib.pyplot as plt
import math
import copy
import tf
from autonomous_eating.msg import command_msg
from random import randint
from random import seed

button_presses = []
mode_select = 0

def input_commands_callback(data):
    global mode_select

    if (data.button1):
        mode_select = 1
    elif (data.button2):
            mode_select = 2
    elif (data.button3):
            mode_select = 3
    elif (data.mode_select):
        mode_select = 4

if __name__ == '__main__':
    global mode_select

    rospy.init_node('itci_tester')

    seed(rospy.get_rostime().secs)

    rospy.sleep(10)

    input_suber = rospy.Subscriber('/input_commands', command_msg, input_commands_callback)

    test_counter = 0
    while not rospy.is_shutdown():
        rospy.sleep(2)
        randomNumber = randint(1, 4)

        mode_select = 0

        rospy.loginfo("Random number is: " + str(randomNumber))
        rospy.loginfo("Button1: 1, Button2: 2, Button3: 3, next_step: 4")

        start_time = rospy.get_time()

        while mode_select == 0:
            rospy.sleep(0.1)

        rospy.loginfo(mode_select)

        button_presses.append( (copy.deepcopy(randomNumber), copy.deepcopy(mode_select), rospy.get_time() - start_time ) )

        if (test_counter > 100):
            break
        test_counter = test_counter + 1

    x_axis = []
    color_axis = []
    true_counter = 0
    time_counter = 0.0
    for x in button_presses:
        x_axis.append( x[2] )
        time_counter = time_counter + x[2]
        if x[0] == x[1]:
            color_axis.append('g')
            true_counter = true_counter + 1
        else:
            color_axis.append('r')

    rospy.loginfo("Detection percentage")
    rospy.loginfo(float(true_counter) / float(len(button_presses)))
    rospy.loginfo("Average time")
    rospy.loginfo(time_counter / float(len(button_presses)))

    for x in range(len(x_axis)):
        plt.plot([x, x], [0, x_axis[x]],  marker='o', linestyle='dashed', color=color_axis[x], linewidth=1, markersize=6)


    with open('/home/ubuntu/Desktop/saveFile', 'w') as f:
        for item in button_presses:
            f.write("{0}\n".format(item))

    plt.savefig('/home/ubuntu/Desktop/plotfile.png')
    plt.show()


        

        



