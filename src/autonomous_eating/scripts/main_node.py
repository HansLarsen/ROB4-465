#! /usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf2_msgs.msg
import sys
import copy
import threading
from std_msgs.msg import String, Bool
from std_msgs.msg import Int32MultiArray, Int32
from autonomous_eating.msg import face_cords, gui_mode, gui_status, command_msg
from enum import Enum
import time

#capture face -> capture bowl -> search bowl -> scoop -> search face -> shove

class MoveitApp():
    def __init__(self):
        self.capture_mode = False
        self.start_up = False
        self.current_mode = 10

        rospy.Subscriber('/input_commands',
                    command_msg,
                    self.input_commands_callback)

        rospy.Subscriber('/status_jaco',
                    Int32MultiArray,
                    self.jaco_status_callback)

        self.move_XY_pub = rospy.Publisher('/move_XY', Int32MultiArray, queue_size=10)
        self.find_face_pub = rospy.Publisher('/find_face_trigger', Bool, queue_size=10)
        self.find_bowl_pub = rospy.Publisher('/find_bowl_trigger', Bool, queue_size=10)
        self.move_to_pub = rospy.Publisher('/move_to', String, queue_size=10)
        self.move_capture_pub = rospy.Publisher("/move_capture", Int32, queue_size=10)
        self.gui_status_pub = rospy.Publisher('/gui_status', gui_status, queue_size=10)
        self.gui_mode_pub = rospy.Publisher('/gui_mode', gui_mode, queue_size=10)

        self.gui_status_message = gui_status()
        rospy.Timer(1, self.updater)
        self.previouse_time = rospy.Time.from_sec(time.time()).to_sec()
        self.command_message = command_msg()

    def input_commands_callback(self, data, topic):
        self.command_message = data

        if (self.capture_mode):
            if rospy.Time.from_sec(time.time()).to_sec() - self.previouse_time > 10:
                self.previouse_time = rospy.Time.from_sec(time.time()).to_sec()
                self.publish_move_XY() 

    def jaco_status_callback(self, data, topic):
        if data.data[0] == 0:
            self.gui_status_message.jaco_status = "Disconnected"
        else:
            self.gui_status_message.jaco_status = "Connected"
        
        if data.data[1] == 1:
            self.gui_status_message.moving_status = "Moving"
        else:
            self.gui_status_message.moving_status = "Stopped"

    def gui_status_pub(self):
        self.gui_status_pub.publish(self.gui_status_message)

    def capture_pos_pub(self, bowlTrue=True):
        if bowlTrue == True:
            self.move_capture_pub.publish(1)
        else:
            self.move_capture_pub.publish(0)

    def startup_capture(self):
        self.start_up = True
        self.gui_status_message.main_status = "Waiting for face capture position"

        rospy.sleep(1)

        while self.command_message.mode_select == False:
            rospy.spin()

        self.capture_pos_pub()

        self.gui_status_message.main_status = "Waiting for bowl capture position"

        rospy.sleep(1)

        while self.command_message.mode_select == False:
            rospy.spin()

        self.capture_pos_pub(False)
        self.start_up = False
        self.current_mode = 0

    def robot_goto(self, gotoString="bowl_search_pos"):
        self.move_to_pub.publish(gotoString)


    def publish_move_XY(self):
        message = Int32MultiArray()
        message.data.append(self.command_message.X)
        message.data.append(self.command_message.Y)

        self.move_XY_pub.publish(message)

    def publish_capture_object(self, bowlTrue=True):
        if (bowlTrue):
            self.find_bowl_pub.publish(True)
        else:
            self.find_face_pub.publish(True)

    def runtime_loop(self):
        if (self.command_message.mode_select == True):
            if (self.current_mode == 0): #Find Bowl
                self.current_mode = 1
                
                self.robot_goto()
                self.capture_mode = True

                rospy.sleep(1)

                while (self.gui_status_message.moving_status == "Moving"):
                    rospy.spin()

                self.publish_capture_object()

            elif (self.current_mode == 2): #Scoop bowl
                self.current_mode == 3

                self.capture_mode = False
                self.robot_goto("mouth")

                rospy.sleep(1)

                while (self.gui_status_message.moving_status == "Moving"):
                    rospy.spin()

                self.robot_goto("mouth2")

            elif (self.current_mode == 3): #Find Face
                self.current_mode = 4

                self.robot_goto("face_seach_pos")
                
                rospy.sleep(1)

                while (self.gui_status_message.moving_status == "Moving"):
                    rospy.spin()

                self.publish_capture_object(bowlTrue=False)
                
            elif (self.current_mode == 4): #Shove food face
                self.current_mode = 5
            
            elif (self.current_mode == 5):
                self.current_mode = 0
               

    def updater(self):
        self.gui_status_pub.publish(self.gui_status_message)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('main_node')

    myMoveitApp = MoveitApp()

    myMoveitApp.startup_capture()

    while not rospy.is_shutdown():
        rospy.spin()