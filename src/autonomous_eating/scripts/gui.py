#!/usr/bin/env python


from Tkinter import *
from ttk import *
from PIL import ImageTk, Image

import rospkg
import rospy
from std_msgs.msg import String
from autonomous_eating.msg import gui_status
from autonomous_eating.msg import gui_mode

pkgPath = rospkg.RosPack().get_path('autonomous_eating')

class GUIApp():
  def gui_mode_callback(self, data):
    self.updateAcitvationBar(data.selectness)
    self.updateItciImg(self.frame, data.x, data.y)

  def gui_status_callback(self, data):
    status = (data.jaco_status, data.camera_status, data.main_status, data.moving_status)
    self.updateStatusText(status)

  def updateItciImg(self, frame, x=25,y=25):
    self.itciCanvas.create_image(0,0, anchor=NW, image=self.itciBaseImg, state=NORMAL)
    self.circleSize = 25
    self.x1 = x - self.circleSize
    self.x2 = x + self.circleSize
    self.y1 = y - self.circleSize
    self.y2 = y + self.circleSize
    self.itciCanvas.create_oval(self.x1,self.y1,self.x2,self.y2, width=5, outline="red")

  def updateAcitvationBar(self, value=0):
    self.activationBar['value'] = value

  def updateStatusText(self, status):
    self.text.configure(state="normal")
    self.text.delete(1.0, END)
    self.text.insert(END, "JACO Connect:  " + status[0] + "\n")
    self.text.insert(END, "Camera status:   " + status[1] + "\n")
    self.text.insert(END, "System status:    " + status[2] + "\n")
    self.text.insert(END, "System Moving:  " + status[3] + "\n")
    self.text.configure(state="disabled")

  def __init__(self, master):
    self.frame = Frame(master)
    self.frame.style = Style()
    self.frame.pack()
    #('clam', 'alt', 'default', 'classic')
    self.frame.style.theme_use("clam")
    master.title("Autonomous Eating")

    #label for status panels below it:
    #self.statusLabel = Label(frame, text="SYSTEM STATUS", width=15, font=("Arial", 12, "bold"))
    #self.statusLabel.grid(column=0,row=0)
    
    #add info panel on the left, top part is status panel
    self.text = Text(self.frame, font=("Arial", 12), width = 50)
    self.text.grid(column=0, row=0)
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


#start of script, set up GUI:
root = Tk()
app = GUIApp(root)

#init ros:
rospy.init_node('display_node')
rospy.Subscriber("/gui_status", gui_status , app.gui_status_callback)
rospy.Subscriber("/gui_mode", gui_mode, app.gui_mode_callback)
 
#loop using Tk instead of ros::spin, does the same thing, but keeps the GUI updating
root.mainloop()
