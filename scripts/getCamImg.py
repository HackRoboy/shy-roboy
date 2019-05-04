#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self, *args, **kwargs):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.callback)

    def callback(self,data):
        rospy.loginfo("Depth image received")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            cv2.imshow('depth_image',cv_image)
            cv2.waitKey(3)

        except CvBridgeError as e:
            print(e)


        
        

def main():
    ic = image_converter()
    
    # cv2.namedWindow('depth_image', cv2.WINDOW_NORMAL)
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.loginfo("Node running.")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()