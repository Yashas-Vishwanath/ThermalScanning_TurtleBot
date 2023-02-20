#! /usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header

# convert depth image to point cloud
class DepthToPC:
    def __init__(self):
        self.bridge = CvBridge()
        self.pc_pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=1)
        self.depth_sub = rospy.Subscriber('camera/depth/image_raw', Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

        # get camera info
        camera_info = rospy.wait_for_message('camera/rgb/camera_info', CameraInfo)
        # get camera matrix
        K = np.array(camera_info.K).reshape((3, 3))
        # get distortion coefficients
        D = np.array(camera_info.D)

        # get image size
        h, w = cv_image.shape

        # get new camera matrix
        new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))

        # undistort image
        mapx, mapy = cv2.initUndistortRectifyMap(K, D, None, new_K, (w, h), 5)
        undistorted_img = cv2.remap(cv_image, mapx, mapy, cv2.INTER_LINEAR)

        # get points
        points = cv2.reprojectImageTo3D(undistorted_img, Q=new_K)

        # get colors
        colors = cv2.cvtColor(undistorted_img, cv2.COLOR_GRAY2RGB)

        # get mask
        mask = undistorted_img > undistorted_img.min()

        # get points
        out_points = points[mask]
        # get colors
        out_colors = colors[mask]
        # get colors
        out_colors = np.hstack([out_colors])

        # create point cloud
        cloud = np.hstack([out_points, out_colors])
        # create point cloud message
        header = Header()
        header.frame_id = "camera_link"
        pc_msg = pc2.create_cloud_xyz32(header, cloud)

        # publish point cloud
        self.pc_pub.publish(pc_msg)

#convert depth image to point cloud using open3d