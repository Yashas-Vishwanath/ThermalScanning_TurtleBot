#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS code to capture images from the Lepton 3.5 and 
# convert the images to aboslute temperature values
# using the Lepton SDK
# and publish the images to a ROS topic

import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy

# run v4l2-ctl --list-devices to determine index
index_ir_cam = 4

bridge = CvBridge()

class Capture:
    
    def __init__(self):
        # initialize ROS node to subscribe to the Lepton 3.5 and publish the images
        rospy.init_node('lepton_pt_capture', anonymous=True)
        self.image_sub = rospy.Subscriber("ir_image", Image, self.callback)
        rospy.loginfo('Node started to receive thermal feed')
        self.img_pub = rospy.Publisher('absolute_ir_image', Image, queue_size=10)
        # rospy.on_shutdown(self.shutdownhook)
        # initialize the Lepton 3.5
        # self.ir_cap = Capture.init_ir_cap()

    def callback(self, data):
        # self.cap_ir = cv2.VideoCapture(index_ir_cam, cv2.CAP_V4L2)
        # print('check 1')
        # cv2.imshow("Image window", self.cap_ir)
        # cv2.waitKey(0)
        # print('check 2')
        # convert ROS image to OpenCV image
        try:
            self.cv_image = bridge.imgmsg_to_cv2(data, "8UC3")
            print('its happening')
        except CvBridgeError as e:
            print(e)
        #view the image
        cv2.imshow("Image window", self.cv_image)
        cv2.waitKey(0)
        # close the window when the 'q' key is pressed
        if cv2.waitKey(0) and 0xFF == ord('q'):
            cv2.destroyAllWindows()
            # release the camera
            self.cap_ir.release()

    # def shutdownhook(self):
    #     self.cap_ir.release()
    #     cv2.destroyAllWindows()

    # def capture(self):
    #     # Capture frame-by-frame
    #     code, frame = self.cap_ir.read()
    #     # If frame is read correctly ret is True
    #     if not code:
    #         rospy.loginfo("Can't receive frame (stream end?). Exiting ...")
    #         exit()
    #     # convert the image to absolute temperature values
    #     frame = frame.astype(np.float32)
    #     frame = (frame * 0.01) - 273.15
    #     # publish the image
    #     self.img_pub.publish(bridge.cv2_to_imgmsg(frame, "mono16"))

if __name__ == '__main__':
    capture = Capture()
    # capture.capture()
    rospy.spin()
