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
        self.current_mode = 0

        rospy.Subscriber('/input_commands',
                    command_msg,
                    self.input_commands_callback)

        rospy.Subscriber('/status_jaco',
                    Int32MultiArray,
                    self.jaco_status_callback)

        self.move_XY_pub = rospy.Publisher('/move_xy', Int32MultiArray, queue_size=10)
        self.find_face_pub = rospy.Publisher('/find_face_trigger', Bool, queue_size=10)
        self.find_bowl_pub = rospy.Publisher('/find_bowl_trigger', Bool, queue_size=10)
        self.move_to_pub = rospy.Publisher('/move_to', String, queue_size=10)
        self.move_capture_pub = rospy.Publisher("/move_capture", Int32, queue_size=10)
        self.gui_status_pub = rospy.Publisher('/gui_status', gui_status, queue_size=10)
        self.gui_mode_pub = rospy.Publisher('/gui_mode', gui_mode, queue_size=10)

        self.old_mode_state = False

        self.previouse_time = time.time()
        self.previouse_time_th1 = time.time()
        self.command_message = command_msg()

        self.gui_status_message = gui_status()
        #rospy.Timer(1, self.updater)
        self.thread_pub = threading.Thread(target=self.updater)
        self.thread_runtime = threading.Thread(target=self.runtime_loop)

        self.thread_pub.start()
        self.thread_runtime.start()

    def input_commands_callback(self, data):
        self.command_message = data

        if (self.capture_mode):
            if time.time() - self.previouse_time > 10:
                self.previouse_time = time.time()
                self.publish_move_XY() 

    def jaco_status_callback(self, data):
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
        message.data.append(self.command_message.x)
        message.data.append(self.command_message.y)

        self.move_XY_pub.publish(message)

    def publish_capture_object(self, bowlTrue=True):
        if (bowlTrue):
            self.find_bowl_pub.publish(True)
        else:
            self.find_face_pub.publish(True)

    def runtime_loop(self):
        while(rospy.is_shutdown() == False):
            
            if (self.old_mode_state != self.command_message.mode_select):
                self.old_mode_state = self.command_message.mode_select
                rospy.loginfo("Currentmode is:") 
                rospy.loginfo(str(self.current_mode))
            elif time.time() - self.previouse_time_th1 < 1.0:
                continue

            self.previouse_time_th1 = time.time()

            if (self.command_message.mode_select == True):
                rospy.loginfo("Got press")
                if (self.current_mode == 0): #Find Bowl
                    self.current_mode = 1

                    rospy.loginfo("Going to the bowl_search_pos and capture mode")
                    
                    self.robot_goto()
                    self.capture_mode = True

                    rospy.sleep(1)

                    while (self.gui_status_message.moving_status == "Moving"):
                        rospy.spin()

                    self.publish_capture_object()

                elif (self.current_mode == 1): #Scoop bowl
                    self.current_mode = 2

                    self.capture_mode = False
                    self.robot_goto("scoop_bowl")

                    rospy.loginfo("Scooping the bowl")

                    rospy.sleep(1)

                    while (self.gui_status_message.moving_status == "Moving"):
                        rospy.spin()

                elif (self.current_mode == 2): #Find Face
                    self.current_mode = 3

                    self.robot_goto("face_seach_pos")

                    rospy.loginfo("Finding the face")
                    
                    rospy.sleep(1)

                    while (self.gui_status_message.moving_status == "Moving"):
                        rospy.spin()

                    self.publish_capture_object(bowlTrue=False)
                    
                elif (self.current_mode == 3): #Shove food face
                    self.current_mode = 4


                    self.capture_mode = False
                    self.robot_goto("mouth")

                    rospy.loginfo("Going to mouth")

                    rospy.sleep(1)

                    while (self.gui_status_message.moving_status == "Moving"):
                        rospy.spin()

                    self.robot_goto("mouth2")
                
                elif (self.current_mode == 4):
                    self.current_mode = 0

                    self.robot_goto("mouth2")

                    rospy.loginfo("Retracting from the mouth")
                

    def updater(self):
        while(rospy.is_shutdown() == False):
            self.gui_status_pub.publish(self.gui_status_message)
            rospy.sleep(1)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('main_node')

    myMoveitApp = MoveitApp()

    #myMoveitApp.startup_capture()

    while not rospy.is_shutdown():
        rospy.spin()