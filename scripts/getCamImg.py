#!/usr/bin/env python
import rospy
import math
import cv2
import numpy as np
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self, *args, **kwargs):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.callback)
        self.distance_pub = rospy.Publisher("shy_roboy/nearest_distance", Float32,queue_size=10)
        self.cv_image = []
        self.callback_received = False

    def callback(self,data):
        # rospy.loginfo("Depth image received")

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")

            cv2.imshow('depth_image',self.cv_image)
            cv2.waitKey(3)

            self.callback_received = True

        except CvBridgeError as e:
            print(e)

    def getDepth(self):
        height, width = self.cv_image.shape
        depth = self.cv_image[(int)(height/2),(int)(width/2)]
        #rospy.loginfo("height, width: {},{}".format(height,width))


        #rospy.loginfo("depth of center: {}".format(depth))
        
        #publish if distance is not nan
        if (not math.isnan(depth) and not np.isinf(depth)):
            self.distance_pub.publish(depth)


        
        

def main():
    ic = image_converter()
    
    # cv2.namedWindow('depth_image', cv2.WINDOW_NORMAL)
    rospy.init_node('image_converter', anonymous=True)
    rospy.loginfo("Node running.")
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():

            if ic.callback_received:
                ic.getDepth()

            rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()