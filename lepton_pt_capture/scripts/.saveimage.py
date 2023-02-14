#! /usr/bin/python3
import rospy
#from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

#image subcriber node for the lepton camera
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I saw %s", data.data)
    print('press 1 to click image')
    #take input from user
    input = int(raw_input())
    if input == 1:
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        cv2.imwrite('test.png', cv_image)
        print('image saved')
    

def listener():
    rospy.init_node('lepton_subcriber', anonymous=True)
    rospy.Subscriber("ir_image", Image, callback)
    rospy.spin()