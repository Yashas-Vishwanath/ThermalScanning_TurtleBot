#! /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import time
#from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

#subscribe to the image from topic ir_image and save it to the directory ThermalCaptures
class ImageSave:
    #initialize the node based on key press
    def __init__(self):
        # if int(input("Type 0 to initialize node: ")) == 0:
        #     rospy.init_node('lepton_pt_save', anonymous=True)
        #     rospy.loginfo("Node received image")
        #     self.image_sub = rospy.Subscriber("ir_image", Image, self.callback)
        # else:
        #     #rerun the def __init__ function
        #     self.__init__()
        ###rospy.init_node('lepton_pt_save', anonymous=True)
        ###rospy.loginfo("Node 2 received image")
        self.image_sub = rospy.Subscriber("ir_image", Image, self.callback)
    
    def callback(self, data):
        #get time stamp
        now = time.strftime("%Y-%m-%d-%H-%M-%S")
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "8UC3")
        except CvBridgeError as e:
            print(e)
        # #save the image to the directory ThermalCaptures
        # # if int(input("Type 1 to save image: ")) == 1:
        # cv2.imwrite('/home/yashas/Term2/Studio/ThermalCaptures/%s.png'%now, cv_image)
        # print('image saved')
        # #rerun ImageSave() function
        # self.__init__()
        # save the image only once to the directory ThermalCaptures and then exit
        cv2.imwrite('/home/yashas/Term2/Studio/ThermalCaptures/%s.png'%now, cv_image)
        time.sleep(1)
        print('Image stored')
        # rospy.on_shutdown(ImageSave)
        #exit the node
        self.image_sub.unregister()
        # rospy.init_node('lepton_pt_save', anonymous=True)

def main():
    ic = ImageSave()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()