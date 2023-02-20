#! /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

# subcribe to the images from topics /camera/color/image_raw and /camera/depth/image_raw and save them in directories RGBCaptures and DepthCaptures with timestamp
class ImageSave:
    def __init__(self):
        # rospy.init_node('astra_save', anonymous=True)  ### comment out these two lines if you want to run this node in the same file as the node that displays the images
        # rospy.loginfo("Node received images")
        self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback1)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback2)

    # callback function for rgb image
    def callback1(self, data):
        now = time.strftime("%Y%m%d-%H%M%S")
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imwrite("/home/yashas/Term2/Studio/RGBCaptures/rgb%s.jpg"%now, cv_image)
        cv2.waitKey(1)
        print("RGB image saved")
        self.rgb_sub.unregister()

    # callback function for depth image
    def callback2(self, data):
        now = time.strftime("%Y%m%d-%H%M%S")
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)
        cv2.imwrite("/home/yashas/Term2/Studio/DepthCaptures/depth%s.jpg"%now, cv_image)
        cv2.waitKey(1)
        print("Depth image saved")
        self.depth_sub.unregister()

    # stop running the node
    def shutdown(self):
        rospy.loginfo("Stopping the node")
        cv2.destroyAllWindows()
        rospy.on_shutdown()

def main():
    ic = ImageSave()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
