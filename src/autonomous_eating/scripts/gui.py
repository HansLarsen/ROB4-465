#!/usr/bin/env python


from Tkinter import *
from ttk import *
from PIL import ImageTk, Image

import rospkg

pkgPath = rospkg.RosPack().get_path('autonomous_eating')

class App():

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
    self.text.insert(END, "JACO Connect:  " + status[0] + "\n")
    self.text.insert(END, "Camera status:   " + status[1] + "\n")
    self.text.insert(END, "System status:    " + status[2] + "\n")
    self.text.insert(END, "System Moving:  " + status[3] + "\n")
    self.text.configure(state="disabled")

  def __init__(self, master):
    frame = Frame(master)
    frame.style = Style()
    frame.pack()
    #('clam', 'alt', 'default', 'classic')
    frame.style.theme_use("clam")
    master.title("Autonomous Eating")

    #label for status panels below it:
    #self.statusLabel = Label(frame, text="SYSTEM STATUS", width=15, font=("Arial", 12, "bold"))
    #self.statusLabel.grid(column=0,row=0)
    
    #add info panel on the left, top part is status panel
    self.text = Text(frame, font=("Arial", 12), width = 50)
    self.text.grid(column=0, row=0)
    self.updateStatusText(("Connected", "Disconnected", "OK", "False"))

    #adding itci image with red ellipse showing tongue position
    self.itciBaseImg = ImageTk.PhotoImage(Image.open(pkgPath + "/extra/figures/itci.png"))
    self.itciCanvas = Canvas(frame, width = self.itciBaseImg.width(), height = self.itciBaseImg.height())
    self.itciCanvas.grid(column=1, row=0, rowspan=2)
    self.updateItciImg(frame) #center ellipse top-left at start

    #add activationbar showing 'activationness' of the itci
    self.activationBar = Progressbar(frame, length=350)
    self.activationBar.grid(column=1,row=2)
    self.updateAcitvationBar()

#start of script:
root = Tk()
app = App(root)
root.mainloop()

