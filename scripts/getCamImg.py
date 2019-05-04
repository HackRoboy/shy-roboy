#!/usr/bin/env python

import rospy
import math
import cv2
import numpy as np
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.callback)
        self.distance_pub = rospy.Publisher("shy_roboy/nearest_distance", Float32, queue_size=10)
        self.cv_image = []
        self.callback_received = False

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.callback_received = True
        except CvBridgeError as e:
            print(e)

    def get_depth(self):
        height, width = self.cv_image.shape

        h_new = height * 0.3
        w_new = width * 0.3

        resized = cv2.resize(self.cv_image, (int(w_new), int(h_new)), interpolation=cv2.INTER_AREA)

        cv2.imshow('Resized Image', resized)
        cv2.waitKey(1)

        _, th = cv2.threshold(resized, 0.7, 1, cv2.THRESH_BINARY_INV)
        cv2.imshow('Thresholded Image', th)
        cv2.waitKey(1)

        if sum(sum(th)) == 0:
            depth = 100.0
        else:
            depth = sum(resized[th == 1]) / sum(sum(th))

        rospy.loginfo("mean depth: {}".format(depth))

        # publish if distance is not nan
        if not math.isnan(depth) and not np.isinf(depth):
            self.distance_pub.publish(depth)


def main():
    ic = ImageConverter()

    rospy.init_node('image_converter', anonymous=True)
    rospy.loginfo("Node running.")

    rate = rospy.Rate(30)

    try:
        while not rospy.is_shutdown():

            if ic.callback_received:
                ic.get_depth()

            rate.sleep()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
