#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from autonomous_eating.msg import face_cords
from std_msgs.msg import Float32MultiArray
import copy

markerArray = MarkerArray()

marker = Marker()
marker.header.frame_id = 'color'
marker.ns = "marker"
marker.type = marker.SPHERE
marker.action = marker.ADD
marker.scale.x = 0.01
marker.scale.y = 0.01
marker.scale.z = 0.01
marker.color.a = 1.0
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0




def face_cords_callback(data):

    marker.header.stamp = rospy.Time.now()
    marker.pose.position.x = data.x_p1/1000
    marker.pose.position.y = data.y_p1/1000
    marker.pose.position.z = data.z_p1/1000
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.id = 0
    markerArray.markers.append(marker)


    marker2 = copy.deepcopy(marker)
    
    marker2.header.stamp = rospy.Time.now()
    marker2.pose.position.x = data.x_p2/1000
    marker2.pose.position.y = data.y_p2/1000.0
    marker2.pose.position.z = data.z_p2/1000
    marker2.color.r = 0.0
    marker2.color.g = 1.0
    marker2.color.b = 0.0
    marker2.id = 1
    markerArray.markers.append(marker2)

    pub.publish(markerArray)
    markerArray.markers.pop(0)
    markerArray.markers.pop(0)


def bowl_cords_callback(data):
    global marker
    marker3 = copy.deepcopy(marker)
    marker3.header.stamp = rospy.Time.now()
    marker3.pose.position.x = data.data[0]/1000
    marker3.pose.position.y = data.data[1]/1000
    marker3.pose.position.z = data.data[2]/1000
    marker3.color.r = 1.0
    marker3.color.g = 0.0
    marker3.color.b = 0.0
    marker3.id = 2

    markerArray.markers.append(marker3)
    pub.publish(markerArray)

    markerArray.markers.pop(0)




if __name__ == '__main__':
    rospy.init_node('register')

    pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    rospy.Subscriber('/face_cords',face_cords,face_cords_callback)
    rospy.Subscriber('/bowl_cords',Float32MultiArray,bowl_cords_callback)

    while not rospy.is_shutdown():

        rospy.spin()
        


