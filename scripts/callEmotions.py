#!/usr/bin/env python

import sys
import rospy
from roboy_control_msgs.srv import ShowEmotion
from std_msgs.msg import String


def main():
    rospy.wait_for_service('/roboy/cognition/face/emotion')
    try:
        face_emotion = rospy.ServiceProxy('/roboy/cognition/face/emotion', ShowEmotion)
        response = face_emotion("emotion: 'angry'")
        print("Response: {}".format(response.success))
        return True
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == "__main__":
    main()