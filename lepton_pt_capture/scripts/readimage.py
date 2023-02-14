#! /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

#publish the image from the topic ir_image to the topic ir_image_save
class ImagePublish:
    def callback(self, data):
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "8UC3")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)
        #self.image_pub.publish(data)

    def __init__(self):
        rospy.init_node('lepton_pt_view', anonymous=True)
        rospy.loginfo("Node 1 started receiving thermal feed")
        self.image_sub = rospy.Subscriber("ir_image", Image, self.callback)
        # #publish the image to the topic ir_image_save with key press
        # q = int(input("Press 0 to save image: "))
        # if q == 0:
        self.image_pub = rospy.Publisher("ir_image_save", Image, queue_size=10)
        print("Image published")
        # else:
        #     print("Image not published")

    # def saveimage():
    #     now = time.strftime("%Y-%m-%d-%H-%M-%S")
    #     try:
    #         cv_image = bridge.imgmsg_to_cv2("8UC3")
    #     except CvBridgeError as e:
    #         print(e)
    #     cv2.imwrite('/home/yashas/Term2/Studio/ThermalCaptures/%s.png'%now, cv_image)
    #     print('Image stored')
    #     #exit the node
    #     rospy.signal_shutdown("Image saved")

    #stop running the class ImagePublish when the node is shutdown
    def shutdown(self):
        rospy.loginfo("Stopping the node")
        #stop the node
        cv2.destroyAllWindows()
        rospy.on_shutdown()


def main():
    ic = ImagePublish()  #what is this line for?
    try:
        rospy.spin()
    except KeyboardInterrupt & ImagePublish.shutdown():
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()