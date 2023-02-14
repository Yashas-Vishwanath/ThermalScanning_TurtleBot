#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np

import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# run v4l2-ctl --list-devices to determine index
index_ir_cam = 4


class Capture:

    def __init__(self):
        rospy.init_node('lepton_pt_capture', anonymous=True)
        rospy.on_shutdown(self.shutdownhook)
        self.img_pub = rospy.Publisher('ir_image', Image, queue_size=10)
        self.bridge = CvBridge()
        self.ir_cap = Capture.init_ir_cap()


        rospy.loginfo('%s started' % rospy.get_name())

    @staticmethod
    def init_ir_cap():
        # use V4l2 pipe otherwise CV2 returns 8bit image
        cap_ir = cv2.VideoCapture(index_ir_cam, cv2.CAP_V4L2)
        # camera can capture in 2 different formats with openCV Y16 (raw sensor values) and BGR3 (colorized by frame)
        # raw format
        # cap_ir.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y', '1', '6', ' '))
        # colored format
        cap_ir.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('B', 'G', 'R', '3'))
        cap_ir.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        # force resolution otherwise weirdness happens (extra pixels)
        cap_ir.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
        cap_ir.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
        cap_ir.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap_ir.isOpened():
            rospy.logerr("Cannot open ir camera")
            exit()

        return cap_ir

    def shutdownhook(self):
        self.ir_cap.release()
        cv2.destroyAllWindows()

    def capture(self):

        while True:
            # Capture frame-by-frame
            code, frame = self.ir_cap.read()
            # If frame is read correctly ret is True
            if not code:
                rospy.loginfo("Can't receive frame (stream end?). Exiting ...")
                break

            # Operations on frames go here
            # Resize image it matches the size of depth cameras
            frame = cv2.resize(frame[:, :], (640, 480))
            # For turtlebot frame should rotate 180
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            # Uncomment the line below if you are capturing raw but want to send a visible image(usefull for debugging)
            # frame = Capture.raw_to_8bit(frame)

            try:
                # Use for color or grayscale 8bit
                i_msg = self.bridge.cv2_to_imgmsg(frame,'8UC3')

                # Use for raw 16 bit format
                # i_msg = self.bridge.cv2_to_imgmsg(frame, 'mono16')

                i_msg.header.stamp = rospy.rostime.Time.now()
                # Frame for realsense
                # i_msg.header.frame_id = 'camera_depth_optical_frame'
                # Frame for astra
                i_msg.header.frame_id = 'camera_rgb_optical_frame'
                self.img_pub.publish(i_msg)

            except CvBridgeError as e:
                rospy.logerr(e)

    def raw_to_8bit(data):
        cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
        np.right_shift(data, 8, data)
        return cv2.cvtColor(np.uint8(data), cv2.COLOR_GRAY2RGB)


def main():
    cap = Capture()
    cap.capture()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('%s stopped' % rospy.get_name())


if __name__ == '__main__':
    main()
