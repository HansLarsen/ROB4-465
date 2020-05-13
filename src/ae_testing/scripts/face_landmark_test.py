#! /usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.srv import GetModelState
from scipy import stats
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
        self.rootFrame = 'world'
        self.cameraNameFrame = 'color_corrected_frame'

        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.Subscriber('/face_cords',
                    face_cords,
                    self.cords_callback)

        self.counter = 0
        self.listedInfo = False
        self.poses = []
        self.x = []
        self.y = []
        self.z = []
        self.sampleSize = 1000.0

    def cords_callback(self, data):
        if self.counter < self.sampleSize:
            if self.counter % (self.sampleSize/20) == 0.0:
                print (self.counter/self.sampleSize)*100.0,
                print "%% done sampling"
                pass

            self.counter = self.counter + 1

           

            target_pose = geometry_msgs.msg.PoseStamped()

            target_pose.pose.position.x = data.x_p2/1000.0
            target_pose.pose.position.y = data.y_p2/1000.0
            target_pose.pose.position.z = data.z_p2/1000.0
            target_pose.header.frame_id = self.cameraNameFrame
            target_pose.header.stamp = rospy.Time.now()

            try:
                camera_transforms = self.tfBuffer.lookup_transform(self.rootFrame, self.cameraNameFrame, rospy.Time(0))

                target_transformed_pose = tf2_geometry_msgs.do_transform_pose(target_pose, camera_transforms)

                #rospy.loginfo(target_transformed_pose.pose.position)
                self.poses.append(target_transformed_pose.pose.position)
                self.x.append(target_transformed_pose.pose.position.x)
                self.y.append(target_transformed_pose.pose.position.y)
                self.z.append(target_transformed_pose.pose.position.z)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.spin()
                rospy.loginfo("Failed lookup")
                return

            


        else:
            if not self.listedInfo:
                #show output
                self.listedInfo = True

                
                #print(poseArray)
                #np.savetxt("1.csv", poseArray, delimiter=',')

                #get actual face position from gazebo service:
                try:
                    service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                    req = GetModelState._request_class()

                    req.model_name = 'human'
                    req.relative_entity_name = ''
                

                    resp = service(req)

                except rospy.ServiceException, e:
                    print("Service call failed: %s" %e)

                print(resp.pose.position)

                poseArray = np.asarray(self.poses)
                xArray = (np.asarray(self.x)-resp.pose.position.x)*1000.0
                yArray = (np.asarray(self.y)-resp.pose.position.y)*1000.0
                zArray = (np.asarray(self.z)-resp.pose.position.z)*1000.0

                meanX, sigmaX = np.mean(xArray), np.std(xArray)
                conf_intX = stats.norm.interval(0.95, loc=meanX, scale=sigmaX/np.sqrt(len(xArray)))
                conf_intX = conf_intX[0] - conf_intX[1]
                meanY, sigmaY = np.mean(yArray), np.std(yArray)
                conf_intY = stats.norm.interval(0.95, loc=meanY, scale=sigmaY/np.sqrt(len(yArray)))
                conf_intY = conf_intY[0] - conf_intY[1]
                meanZ, sigmaZ = np.mean(zArray), np.std(zArray)
                conf_intZ = stats.norm.interval(0.95, loc=meanZ, scale=sigmaZ/np.sqrt(len(zArray)))
                conf_intZ = conf_intZ[0] - conf_intZ[1]



                print "X: ",
                print round(meanX, 1),
                print '+-',
                print round(abs(conf_intX), 1),
                print ', std.dev: ',
                print round(sigmaX, 1)

                print "Y: ",
                print round(meanY, 1),
                print '+-',
                print round(abs(conf_intY), 1),
                print ', std.dev: ',
                print round(sigmaY, 1)
                
                print "Z: ",
                print round(meanZ, 1),
                print '+-',
                print round(abs(conf_intZ), 1),
                print ', std.dev: ',
                print round(sigmaZ, 1)

            



if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_node')

    myMoveitApp = MoveitApp()
    rospy.spin()