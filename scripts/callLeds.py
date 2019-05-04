#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Int8
import time

ROS_PUBLISHER_NAME = '/roboy/control/matrix/leds/mode/simple'

RED_AND_BLUE_LED = Int32(3)
SHUT_DOWN_LED = Int32(0)

last_publish_time = time.time()
previous_state = 1

def callback(data):
    rospy.loginfo('Initializing and starting ' + ROS_PUBLISHER_NAME + ' topic.')
    
    # ROS publisher
    publisher = rospy.Publisher(ROS_PUBLISHER_NAME, Int32, queue_size=10)

    rospy.loginfo(ROS_PUBLISHER_NAME + ' topic has been initialized.')

    if (data.data == 2 or data.data == 3) and (time.time()-last_publish_time) > 6:
        rospy.loginfo('Sending change LED command.')
        previous_state = data.data
        last_publish_time = time.time()
        publisher.publish(RED_AND_BLUE_LED)
    elif (data.data == 1 or data.data == 4) and (previous_state == 2 or previous_state == 3):
        rospy.loginfo('Sending turn off LED command.')
        previous_state = data.data
        publisher.publish(SHUT_DOWN_LED)


def listener():
    rospy.init_node('led_activate_listener', anonymous=True)

    rospy.Subscriber('shy_roboy/state', Int8, callback)
    
    # Communication rate
    rate = rospy.Rate(30) # 60 Hz = 30 FPS
    
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    try:
        listener()
    except Exception as e:
        print e