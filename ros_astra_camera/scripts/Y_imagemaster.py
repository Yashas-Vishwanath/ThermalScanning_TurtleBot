#! /usr/bin/python3
# -*- coding: utf-8 -*-

from Y_readimage import ImageView
from Y_storeimage import ImageSave
import tkinter as tk

def onclick(arg):
    if arg == 1:
        print('button 1 clicked')
        #call the class ImageView
        ImageView()

    if arg == 2:
        print('button 2 clicked')
        ImageSave()

root = tk.Tk()
root.title("GUI Button")

btn1 = tk.Button(root, text="View", command=lambda:onclick(1))
btn2 = tk.Button(root, text="Capture", command=lambda:onclick(2))

btn1.pack()
btn2.pack()

root.mainloop()
