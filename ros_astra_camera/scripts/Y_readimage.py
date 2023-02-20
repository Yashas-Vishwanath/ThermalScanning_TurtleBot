#! /usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

#subscribe to the images from topics /camera/color/image_raw and /camera/depth/image_raw and display them
class ImageView:
    def __init__(self):
        rospy.init_node('astra_view', anonymous=True)
        rospy.loginfo("Node received images")
        self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback1)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback2)

    # callback function for rgb image
    def callback1(self, data):
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("RGB Image", cv_image)
        cv2.waitKey(1)

    # callback function for depth image
    def callback2(self, data):
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Depth Image", cv_image)
        cv2.waitKey(1)

    # stop running the node
    def shutdown(self):
        rospy.loginfo("Stopping the node")
        cv2.destroyAllWindows()
        rospy.on_shutdown()

def main():
    ic = ImageView()
    try:
        rospy.spin()
    except KeyboardInterrupt or ImageView.shutdown():
        print("Shutting down")
    cv2.destroyAllWindows()
