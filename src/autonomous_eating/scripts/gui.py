#!/usr/bin/env python


from sensor_msgs.msg import Image as ROS_Image

#packages for the GUI
from Tkinter import *
from ttk import *
from PIL import ImageTk, Image

import rospkg
import rospy
from cv_bridge import CvBridge
import cv2 as cv2

#import msg's from our package
from autonomous_eating.msg import gui_status
from autonomous_eating.msg import gui_mode

pkgPath = rospkg.RosPack().get_path('autonomous_eating')

#has all the methods and handles for the GUI
class GUIApp():
  #callback for gui_mode
  def gui_mode_callback(self, data):
    self.updateAcitvationBar(data.selectness)
    self.updateItciImg(self.frame, data.x, data.y)

  #callback for gui_status
  def gui_status_callback(self, data):
    status = (data.jaco_status, data.camera_status, data.main_status, data.moving_status)
    self.updateStatusText(status)

  def gui_figure_callback(self, data):
    bridge = CvBridge()
    self.cv_img = bridge.imgmsg_to_cv2(data)
    self.pil_img = ImageTk.PhotoImage(image=Image.fromarray(self.cv_img))
    self.figureCanvas.create_image(0,0, anchor=NW, image=self.pil_img, state=NORMAL)

  #updates image of itci with the red circle showing tongue position
  def updateItciImg(self, frame, x=25,y=25):
    self.itciCanvas.create_image(0,0, anchor=NW, image=self.itciBaseImg, state=NORMAL)
    self.circleSize = 25
    self.x1 = x - self.circleSize
    self.x2 = x + self.circleSize
    self.y1 = y - self.circleSize
    self.y2 = y + self.circleSize
    self.itciCanvas.create_oval(self.x1,self.y1,self.x2,self.y2, width=5, outline="red")

  #shows selectiveness as a status bar
  def updateAcitvationBar(self, value=0):
    self.activationBar['value'] = value

  #updates text box with the status messages
  def updateStatusText(self, status):
    self.text.configure(state="normal") #unlock text field
    self.text.delete(1.0, END)
    self.text.insert(END, "JACO Connect:  " + status[0] + "\n")
    self.text.insert(END, "Camera status:   " + status[1] + "\n")
    self.text.insert(END, "System status:    " + status[2] + "\n")
    self.text.insert(END, "System Moving:  " + status[3] + "\n")
    self.text.configure(state="disabled") #lock it, so user does not mess with it

  #init the gui, place widgets in grid, and setup the initial state
  def __init__(self, master):
    self.frame = Frame(master)
    self.frame.style = Style()
    self.frame.pack()
    self.frame.style.theme_use("clam")
    master.title("Autonomous Eating")

    #add info panel on the left
    self.text = Text(self.frame, font=("Arial", 12), width = 50)
    self.text.grid(column=0, row=0)
    # set text at start:
    self.updateStatusText(("Connected", "Disconnected", "OK", "False"))

    #adding itci image with red ellipse showing tongue position
    self.itciBaseImg = ImageTk.PhotoImage(Image.open(pkgPath + "/extra/figures/itci.png"))
    self.itciCanvas = Canvas(self.frame, width = self.itciBaseImg.width(), height = self.itciBaseImg.height())
    self.itciCanvas.grid(column=1, row=0, rowspan=2)
    self.updateItciImg(self.frame) #center ellipse top-left at start

    #add activationbar showing 'activationness' of the itci
    self.activationBar = Progressbar(self.frame, length=350)
    self.activationBar.grid(column=1,row=2)
    self.updateAcitvationBar()

    #add figure to show on the right (from /gui_figure topic)
    self.figureCanvas = Canvas(self.frame, width = 500, height = 500)
    self.figureCanvas.grid(column=2, row=0, rowspan=2)

    rospy.Subscriber("/gui_status", gui_status, self.gui_status_callback)
    rospy.Subscriber("/gui_mode", gui_mode, self.gui_mode_callback)
    rospy.Subscriber("/gui_figure", ROS_Image, self.gui_figure_callback)



#start of script, set up GUI:
root = Tk()
app = GUIApp(root)

#init ros:
rospy.init_node('display_node')
#loop using Tk instead of ros::spin, does the same thing, but keeps the GUI updating
root.mainloop()
