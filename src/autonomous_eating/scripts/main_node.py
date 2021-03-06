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
from sensor_msgs.msg import Image
import time

#capture face -> capture bowl -> search bowl -> scoop -> search face -> shove

class MoveitApp():
    def __init__(self):
        self.capture_mode = False
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
        self.color_sub = rospy.Subscriber("r200/camera/color/image_raw", Image, self.color_camera_calback)

        self.old_mode_state = False
        self.old_mode_state1 = False
        self.old_mode_state2 = False
        self.old_mode_state3 = False
        self.mouth_mode = False

        self.previouse_time = time.time()
        self.previouse_time_th1 = time.time()
        self.command_message = command_msg()

        self.gui_status_message = gui_status()

        self.gui_status_message.jaco_status = "Disconnected"
        self.gui_status_message.moving_status = "Starting"
        self.gui_status_message.main_status = "Starting"
        self.gui_status_message.camera_status = "Starting"

        self.thread_pub = threading.Thread(target=self.updater)
        self.thread_runtime = threading.Thread(target=self.runtime_loop)

        self.thread_pub.start()
        self.thread_runtime.start()

    def color_camera_calback(self, data):
        self.gui_status_message.camera_status = "Connected"
        self.color_sub.unregister()

    def input_commands_callback(self, data):
        self.command_message = data

        if (self.capture_mode):
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
            self.find_bowl_pub.publish(False)


    def publish_face_capture_object(self, faceTrue=True):
        if (faceTrue):
            self.find_face_pub.publish(True)
        else:
            self.find_face_pub.publish(False)

    def timeout_check(self):
        if (time.time() - self.timeout_check_var > 10.0):
            rospy.loginfo("Timeout on movement")
            return False
        else:
            return True


    def runtime_loop(self):
        while(rospy.is_shutdown() == False):
            if (self.mouth_mode == True):
                if (self.old_mode_state3 != self.command_message.button3):
                    self.old_mode_state3 = self.command_message.button3

                    if (self.old_mode_state3 == True):
                        self.robot_goto("mouth2")
                    else:
                        self.robot_goto("stop")

            if (self.old_mode_state2 != self.command_message.button2):
                self.old_mode_state2 = self.command_message.button2

                if(self.old_mode_state2 == False):
                    continue
                
                if (self.current_mode == 0): #Find Bowl
                    self.current_mode = 1

                    rospy.loginfo("Going to the bowl_search_pos and capture mode")
                    self.gui_status_message.main_status = "Going to bowl search position"
                    
                    self.robot_goto("bowl_search_pos")
                    self.capture_mode = True

                    rospy.sleep(1)

                    self.timeout_check_var = time.time()
                    while (self.gui_status_message.moving_status == "Moving" and self.timeout_check()):
                        rospy.sleep(1)

                    self.publish_capture_object()
                    self.gui_status_message.main_status = "Waiting, move arm to find bowl"

                elif (self.current_mode == 1): #Scoop bowl
                    self.current_mode = 0

                    self.gui_status_message.main_status = "Going for the scoop"

                    self.capture_mode = False

                    self.publish_capture_object(False)

                    rospy.sleep(5)

                    self.robot_goto("scoop_bowl")

                    self.gui_status_message.main_status = "Retracting"

                    rospy.loginfo("Scooping the bowl")

                    rospy.sleep(5)

                    self.timeout_check_var = time.time()
                    while (self.gui_status_message.moving_status == "Moving" and self.timeout_check()):
                        rospy.sleep(1)

                    rospy.loginfo("Finished scooping")
                    self.gui_status_message.main_status = "Waiting"


            elif (self.old_mode_state1 != self.command_message.button1):
                self.old_mode_state1 = self.command_message.button1

                if(self.old_mode_state1 == False):
                    continue

                self.capture_mode = False

                self.current_mode = 3

                self.robot_goto("face_search_pos")

                rospy.loginfo("Finding the face")
                
                rospy.sleep(1)

                self.timeout_check_var = time.time()
                while (self.gui_status_message.moving_status == "Moving" and self.timeout_check()):
                    rospy.sleep(1)

                self.publish_face_capture_object(True)

            elif (self.old_mode_state != self.command_message.mode_select):
                self.old_mode_state = self.command_message.mode_select

                if(self.old_mode_state == False):
                    continue

                rospy.loginfo("Got press")
                if (self.current_mode == 0): #Find Bowl
                    self.current_mode = 1

                    self.mouth_mode = False

                    rospy.loginfo("Going to the bowl_search_pos and capture mode")
                    self.gui_status_message.main_status = "Going to bowl search position"
                    
                    self.robot_goto("bowl_search_pos")
                    self.capture_mode = True

                    rospy.sleep(1)

                    self.timeout_check_var = time.time()
                    while (self.gui_status_message.moving_status == "Moving" and self.timeout_check()):
                        rospy.sleep(1)

                    self.publish_capture_object()
                    self.gui_status_message.main_status = "Waiting, move arm to find bowl"

                elif (self.current_mode == 1): #Scoop bowl
                    #self.current_mode = 2

                    self.gui_status_message.main_status = "Going for the scoop"

                    self.capture_mode = False

                    self.publish_capture_object(False)

                    rospy.sleep(5)

                    self.robot_goto("scoop_bowl")

                    self.gui_status_message.main_status = "Retracting"

                    rospy.loginfo("Scooping the bowl")

                    rospy.sleep(15)

                    self.timeout_check_var = time.time()
                    while (self.gui_status_message.moving_status == "Moving" and self.timeout_check()):
                        rospy.sleep(1)

                    self.robot_goto("face_search_pos")

                    rospy.loginfo("Finished scooping")
                    self.gui_status_message.main_status = "Waiting"

                #elif (self.current_mode == 2): #Find Face
                    self.current_mode = 3

                    self.gui_status_message.main_status = "Searching for the face"
                    #self.robot_goto("face_search_pos")

                    rospy.loginfo("Finding the face")
                    
                    rospy.sleep(1)

                    self.timeout_check_var = time.time()
                    while (self.gui_status_message.moving_status == "Moving" and self.timeout_check()):
                        rospy.sleep(1)

                    self.publish_face_capture_object(True)
                    self.gui_status_message.main_status = "Waiting"
                    
                elif (self.current_mode == 3): #Shove food face
                    self.current_mode = 0

                    self.publish_face_capture_object(False)
                    self.gui_status_message.main_status = "Going to the mouth"

                    rospy.sleep(2)

                    self.robot_goto("mouth")

                    rospy.loginfo("Going to mouth")

                    rospy.sleep(1)

                    self.timeout_check_var = time.time()
                    while (self.gui_status_message.moving_status == "Moving" and self.timeout_check()):
                        rospy.sleep(1)

                    self.mouth_mode = True
                    self.gui_status_message.main_status = "Waiting for button 3"

                    #self.robot_goto("mouth2")
                    #self.gui_status_message.main_status = "Waiting for retract command"
                
                #elif (self.current_mode == 4):
                #    self.current_mode = 0

                #    self.robot_goto("mouth3")

                #    self.gui_status_message.main_status = "Waiting"

                #   rospy.loginfo("Retracting from the mouth")
            
            elif time.time() - self.previouse_time_th1 < 1.0:
                continue
                

    def updater(self):
        while(rospy.is_shutdown() == False):
            self.gui_status_pub.publish(self.gui_status_message)
            rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('main_node')

    myMoveitApp = MoveitApp()

    #myMoveitApp.startup_capture()

    rospy.spin()