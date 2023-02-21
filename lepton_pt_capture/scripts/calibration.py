#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import glob
import time

# Define the chessboard pattern size
pattern_size = (9, 6)

###### get image frame size of image to be undistorted ######
# Load an image
# img_forSize = cv2.imread("/home/yashas/cork_ws/src/ThermalScanning_Turtlebot/lepton_pt_capture/calibration_images/2023-02-21-13-36-28.png")
img_forSize = cv2.imread("2023-02-21-13-33-51.png")

# Get the frame size
height, width, channels = img_forSize.shape

# print("Image height: ", height)
# print("Image width: ", width)

frame_size = (width, height)


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Create an array of object points
objp = np.zeros((np.prod(pattern_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

# Define arrays to store object points and image points
objpoints = []
imgpoints = []

# Load calibration images and convert them to grayscale
calibration_images = glob.glob('/home/yashas/cork_ws/src/ThermalScanning_Turtlebot/lepton_pt_capture/calibration_images/*.png')
# gray_images = []
# if len(calibration_images) > 0:
for img_file in calibration_images:
        # Load the image
        img = cv2.imread(img_file)

        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # gray_images.append(gray)

    # Loop through the grayscale images and detect chessboard corners
# for gray in gray_images:
        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None) 

        # convert onject and image points to numpy arrays
        # objpoints_list = []
        # imgpoints_list = []

        # If corners are found, add them to the lists of object and image points
        if ret == True:
            objpoints.append(objp) #add 3D coordinates for this image
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners) #add 2D coordinates for this image

            cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(1000)

cv2.destroyAllWindows()

        # objpoints = np.array(objpoints_list)
        # imgpoints = np.array(imgpoints_list)

        #     # Draw the chessboard corners on the image
        #     img = cv2.drawChessboardCorners(img, pattern_size, corners, ret)

        # # Show the image with chessboard corners
        # cv2.imshow('img', img)
        # cv2.waitKey(500)

    # Check the size of the objpoints and imgpoints arrays
print("Number of calibration images:", len(calibration_images))
print("Number of images with corners detected:", len(objpoints))
print("Size of objpoints array:", np.array(objpoints).shape)
print("Size of imgpoints array:", np.array(imgpoints).shape)

    # Calibrate the camera and obtain intrinsic parameters and distortion coefficients
ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frame_size, None, None)

print("Camera Calibrated: ", ret)
print("\nCamera Matrix: \n", K)
print("\nDistortion Parameters: \n", D)
print("\nRotation Vectors: \n", rvecs)
print("\nTranslation Vectors: \n", tvecs)


####### code to undistort images from youtube ########

img = cv2.imread("/home/yashas/cork_ws/src/ThermalScanning_Turtlebot/lepton_pt_capture/imagesToCorrect/2023-02-15-09-31-23.png")
h,  w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K,D,(w,h),1,(w,h))

# undistort
dst = cv2.undistort(img, K, D, None, newcameramtx)
# crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
now = time.strftime("%Y%m%d-%H%M%S")
cv2.imwrite('home/yashas/cork_ws/src/ThermalScanning_Turtlebot/lepton_pt_capture/undistortedImages/undistorted_' + now + '.jpg',dst)

# undistort with remapping
mapx,mapy = cv2.initUndistortRectifyMap(K,D,None,newcameramtx,(w,h),5)
dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
# crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
now = time.strftime("%Y%m%d-%H%M%S")
cv2.imwrite('home/yashas/cork_ws/src/ThermalScanning_Turtlebot/lepton_pt_capture/undistortedImages/undistorted_' + now + '.jpg',dst)

# reprojection error
mean_error = 0

for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

print( "\ntotal error: {}".format(mean_error/len(objpoints)) )
print("\n\n\n")
