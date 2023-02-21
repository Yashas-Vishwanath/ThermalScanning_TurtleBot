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
img_forSize = cv2.imread("/home/yashas/Term2/Studio/RGBCaptures/UseForCalibration/rgb20230220-113659.jpg")

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
calibration_images = glob.glob('/home/yashas/Term2/Studio/RGBCaptures/UseForCalibration/*.jpg')
# gray_images = []
# if len(calibration_images) > 0:
# tag = 0
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
            # tag += 1
            # cv2.imwrite('/home/yashas/Term2/Studio/RGBCaptures/UseForCalibration/gif_of_calibration/%s.jpg'%str(tag), img)


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


# Number of calibration images: 11
# Number of images with corners detected: 1
# Size of objpoints array: (1, 54, 1, 3)
# Size of imgpoints array: (1, 54, 1, 2)
# Intrinsic parameters (K):
# [[210.63129037  -4.66930866 319.690937  ]
#  [  0.         210.70699909 240.30366602]
#  [  0.           0.           1.        ]]
# Distortion coefficients (D):
# [[-0.43502548]
#  [ 1.8681299 ]
#  [-2.81059195]
#  [ 1.47700463]]




####### Code to fix fisheye images

# import cv2
# import numpy as np

# # Load the distorted image
# distorted_img = cv2.imread('distorted_img.jpg')

# # Define the camera matrix and distortion coefficients
# K = np.array([[ fx, 0, cx],
#               [0, fy, cy],
#               [0,  0,  1]])
# D = np.array([k1, k2, p1, p2])

# # Calculate the optimal new camera matrix
# img_shape = distorted_img.shape[:2]
# new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, img_shape, 1, img_shape)

# # Undistort the image
# undistorted_img = cv2.fisheye.undistortImage(distorted_img, K, D, None, new_K)

# # Save the undistorted image
# cv2.imwrite('undistorted_img.jpg', undistorted_img)



######## Explanation of the code

# In this code snippet, fx, fy, cx, cy, k1, k2, p1, and p2 are the camera intrinsic 
# parameters and distortion coefficients that can be obtained from the camera calibration 
# process. The cv2.getOptimalNewCameraMatrix function calculates the optimal new camera 
# matrix based on the input parameters, and the cv2.fisheye.undistortImage function 
# corrects the fisheye distortion in the image using the camera matrix and distortion 
# coefficients. The resulting undistorted image is then saved to a file.

# Note that the accuracy of the fisheye distortion correction depends on the accuracy of 
# the camera calibration and the quality of the camera's intrinsic parameters and 
# distortion coefficients. Therefore, it's important to calibrate the camera properly 
# before attempting to correct fisheye distortion.


####### code to undistort images from youtube ########

img_list = glob.glob("/home/yashas/Term2/Studio/DepthCaptures/*.jpg")
label = 0
for image in img_list:
    img = cv2.imread(image)
    h,  w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K,D,(w,h),1,(w,h))

    # undistort
    dst = cv2.undistort(img, K, D, None, newcameramtx)
    # crop the image
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    # now = time.strftime("%Y%m%d-%H%M%S")
    # cv2.imwrite('/home/yashas/Term2/Studio/RGBUndistorted/undistorted_' + now + '.jpg',dst)

# # undistort with remapping
# mapx,mapy = cv2.initUndistortRectifyMap(K,D,None,newcameramtx,(w,h),5)
# dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
# # crop the image
# x,y,w,h = roi
# dst = dst[y:y+h, x:x+w]
# now = time.strftime("%Y%m%d-%H%M%S")
# cv2.imwrite('/home/yashas/Term2/Studio/RGBUndistorted/undistorted_' + now + '.jpg',dst)

    # reprojection error
    mean_error = 0

    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
        error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error

    print( "\ntotal error: {}".format(mean_error/len(objpoints)) )
    print("\n\n\n")
    label += 1
    cv2.imwrite('/home/yashas/Term2/Studio/DepthUndistorted/undistorted_' + str(label) + '.jpg',dst)

