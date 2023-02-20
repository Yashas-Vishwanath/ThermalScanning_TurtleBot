#! /usr/bin/python3
# -*- coding: utf-8 -*-


########## NOTES ##########
# 1. The class ImagePublish from readimage.py has the node to publish the image
# 2. The class ImageSave from storeimage.py has the node to save the image
# 3. To exit the program, click on the red cross on the top right corner of the button window

## Could not figure out how to stop the node from running after the button is clicked :/


import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from readimage import ImagePublish
from storeimage import ImageSave

import tkinter as tk

def onclick(arg):
        if arg == 1:
            print('button 1 clicked')
            #call the class ImagePublish
            ImagePublish()

        # if arg == 2:
        #     print('button 2 clicked')
        #     #stop the class ImagePublish from running
        #     rospy.is_shutdown()
        #     #start the def onclick(arg) from the beginning
        #     # ip()
    
        if arg == 3:
            print('button 3 clicked')
            ImageSave()
            # time.sleep(1)
            #restart the loop after the button is clicked
            # rospy.on_shutdown(ImagePublish)
            # ImagePublish()
            # onclick(1)


root = tk.Tk()
root.title("GUI Button")

# first step is to create element
btn1 = tk.Button(root, text="View", command=lambda:onclick(1))
# btn2 = tk.Button(root, text="Button2", command=lambda:onclick(2))
btn3 = tk.Button(root, text="Capture", command=lambda:onclick(3))

# last step is to put element on main window
btn1.pack()
# btn2.pack()
btn3.pack()

root.mainloop()
