#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# import cv2
# import numpy as np
# import time

# # load distorted image
# img = cv2.imread('/home/yashas/Term2/Studio/RGBCaptures/rgb20230219-213605.jpg')

# # intrinsic and distortion parameters after calibration

# # Number of calibration images: 11
# # Number of images with corners detected: 1
# # Size of objpoints array: (1, 54, 1, 3)
# # Size of imgpoints array: (1, 54, 1, 2)
# # Intrinsic parameters (K):
# # [[210.63129037  -4.66930866 319.690937  ]
# #  [  0.         210.70699909 240.30366602]
# #  [  0.           0.           1.        ]]
# # Distortion coefficients (D):
# # [[-0.43502548]
# #  [ 1.8681299 ]
# #  [-2.81059195]
# #  [ 1.47700463]]

# K = np.array([[210.63129037, -4.66930866, 319.690937],
#                 [0, 210.70699909, 240.30366602],
#                 [0, 0, 1]])

# D = np.array([-0.43502548, 1.8681299, -2.81059195, 1.47700463])

# # calculate optimal new camera matrix
# h, w = img.shape[:2] ## (height, width)
# new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))

# # undistort image
# mapx, mapy = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)
# undistorted_img = cv2.remap(img, mapx, mapy, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

# # display images
# cv2.imshow('distorted', img)
# cv2.imshow('undistorted', undistorted_img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# # save undistorted image with timestamp
# now = time.strftime("%c")
# cv2.imwrite('/home/yashas/Term2/Studio/RGBUndistorted/undistorted_' + now + '.jpg', undistorted_img)
# print('Undistorted image saved')


import cv2

# Load an image
img = cv2.imread("/home/yashas/Term2/Studio/RGBCaptures/rgb20230219-213605.jpg")

# Get the frame size
height, width, channels = img.shape

print("Image height: ", height)
print("Image width: ", width)
