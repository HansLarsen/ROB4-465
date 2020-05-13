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

pre_mode_select = False
counter_mode_select = 0

start_time = 0
x_lines = []

def dis_transfrom(firstTransform, secondTransform):
    d_x = firstTransform.transform.translation.x - secondTransform.transform.translation.x
    d_y = firstTransform.transform.translation.y - secondTransform.transform.translation.y
    d_z = firstTransform.transform.translation.z - secondTransform.transform.translation.z

    return math.sqrt( d_x**2 + d_y**2 + d_z**2 )

def input_commands_callback(data):
    global pre_mode_select
    global counter_mode_select
    global start_time
    global x_lines

    if (data.mode_select != pre_mode_select):
        pre_mode_select = data.mode_select

        if (pre_mode_select == False):
            return

        counter_mode_select = counter_mode_select + 1
        x_lines.append(rospy.get_time() - start_time)

if __name__ == '__main__':
    global start_time
    global x_lines

    rospy.init_node('velocity_grapher')

    tfBuffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tfBuffer)

    input_suber = rospy.Subscriber('/input_commands', command_msg, input_commands_callback)

    spoonEffector = 'end_effector_spoon'
    rootFrame = 'j2n6s300_link_base'

    first = True

    last_transform = []

    rospy.sleep(1)

    start_time = rospy.get_time()
    previouse_time = rospy.get_time()

    rospy.sleep(1)

    x_vals = []
    y_vals = []

    y2_vals = []

    y3_vals = []

    capture_angle = False

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        print(counter_mode_select)

        if counter_mode_select > 0:
            capture_angle = True
        elif counter_mode_select > 2:
            capture_angle = False

        if counter_mode_select > 3:
            break

        trans = []
        try:
            trans = tfBuffer.lookup_transform(rootFrame, spoonEffector, rospy.Time(0))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get transform")
            continue
               
        if first:
            last_transform = trans
            first = False
            continue

        current_time = rospy.get_time()

        x_vals.append(current_time - start_time)
        d_t = float(current_time - previouse_time)
        previouse_time = current_time

        print(d_t)
        Velocity = (dis_transfrom(last_transform, trans) / d_t)*1000.0

        (r, p, y) = tf.transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])

        y_vals.append(Velocity)
        y2_vals.append(r * (180/math.pi))
        y3_vals.append(p * (180/math.pi))

        last_transform = trans

    input_suber.unregister()

    fig, ax = plt.subplots(3)

    for xc in x_lines:
        ax[0].vlines(xc, 0, 250, color='r')
        ax[1].vlines(xc, -140, -80, color='r')
        ax[2].vlines(xc, -15, 5, color='r')

    ax[0].plot(x_vals, y_vals)

    ax[0].set(xlabel='time (s)', ylabel='velocity (mm/s)', title='Velocity over time')
    ax[1].set(xlabel='time (s)', ylabel='roll (deg)', title='Spoon rotation')
    ax[2].set(xlabel='time (s)', ylabel='pitch (deg)', title='Spoon rotation')

    ax[0].grid()
    ax[1].grid()
    ax[2].grid()

    ax[1].plot(x_vals, y2_vals)
    ax[2].plot(x_vals, y3_vals)

    plt.show()




        

