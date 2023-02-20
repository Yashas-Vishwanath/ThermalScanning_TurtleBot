#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# import cv2
# import numpy as np
# import glob

# # Define the chessboard pattern size
# pattern_size = (9, 6)

# # Create an array of object points
# objp = np.zeros((np.prod(pattern_size), 3), np.float32)
# objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

# # Define arrays to store object points and image points
# objpoints = []
# imgpoints = []

# # Load calibration images and convert them to grayscale
# calibration_images = glob.glob('/home/yashas/cork_ws/src/ThermalScanning_TurtleBot/lepton_pt_capture/calibration_images/*.png')
# gray_images = []
# if len(calibration_images) > 0:
#     for img_file in calibration_images:
#         # Load the image
#         img = cv2.imread(img_file)

#         # Convert the image to grayscale
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         gray_images.append(gray)

#     # Loop through the grayscale images and detect chessboard corners
#     for gray in gray_images:
#         # Find the chessboard corners
#         ret, corners = cv2.findChessboardCorners(gray, pattern_size)

#         # If corners are found, add object points and image points
#         if ret == True:
#             objpoints.append(objp)
#             imgpoints.append(corners)

#             # Draw the chessboard corners on the image
#             img = cv2.drawChessboardCorners(img, pattern_size, corners, ret)

#         # Show the image with chessboard corners
#         cv2.imshow('img', img)
#         cv2.waitKey(500)

#     # Calibrate the camera and obtain intrinsic parameters and distortion coefficients
#     ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
#         objpoints, imgpoints, gray_images[0].shape[::-1], None, None
#     )

#     print("Intrinsic parameters (K):")
#     print(K)
#     print("Distortion coefficients (D):")
#     print(D)
# else:
#     print("No calibration images found in directory")


import cv2
import numpy as np
import glob

# Define the chessboard pattern size
pattern_size = (16, 9)

# Create an array of object points
objp = np.zeros((np.prod(pattern_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

# Define arrays to store object points and image points
objpoints = []
imgpoints = []

# Load calibration images and convert them to grayscale
calibration_images = glob.glob('/home/yashas/cork_ws/src/ThermalScanning_TurtleBot/lepton_pt_capture/calibration_images/*.png')
gray_images = []
if len(calibration_images) > 0:
    for img_file in calibration_images:
        # Load the image
        img = cv2.imread(img_file)

        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_images.append(gray)

    # Loop through the grayscale images and detect chessboard corners
    for gray in gray_images:
        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size)

        # If corners are found, add object points and image points
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

            # Draw the chessboard corners on the image
            img = cv2.drawChessboardCorners(img, pattern_size, corners, ret)

        # Show the image with chessboard corners
        cv2.imshow('img', img)
        cv2.waitKey(500)

    # Check the size of the objpoints and imgpoints arrays
    print("Number of calibration images:", len(calibration_images))
    print("Number of images with corners detected:", len(objpoints))
    print("Size of objpoints array:", np.array(objpoints).shape)
    print("Size of imgpoints array:", np.array(imgpoints).shape)

    # Calibrate the camera and obtain intrinsic parameters and distortion coefficients
    ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        objpoints, imgpoints, gray_images[0].shape[::-1], None, None
    )

    print("Intrinsic parameters (K):")
    print(K)
    print("Distortion coefficients (D):")
    print(D)
else:
    print("No calibration images found in directory")
