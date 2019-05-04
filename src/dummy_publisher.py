import rospy
from std_msgs.msg import Float32
from random import gauss
import time


ROS_PUBLISHER_NAME = 'shy_roboy/nearest_distance'

def talker():
    rospy.loginfo('Initializing and starting ' + ROS_PUBLISHER_NAME + ' topic.')

    # ROS publisher
    publisher = rospy.Publisher(ROS_PUBLISHER_NAME, Float32, queue_size=10)
    
    # Initialization
    rospy.init_node('dummy_distance_publisher')

    # Communication rate
    rate = rospy.Rate(60) # 60 Hz = 30 FPS

    rospy.loginfo(ROS_PUBLISHER_NAME + ' topic has been initialized.')

    while not rospy.is_shutdown():

        mean = 0.5 if int(time.time()) % 10 > 5 else 3.0
        distance = mean + gauss(0, 0.1)
        rospy.loginfo(distance)
        publisher.publish(distance)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except Exception as e:
        print(e)
