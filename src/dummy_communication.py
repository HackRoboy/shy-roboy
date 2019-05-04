import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from random import randint


ROS_PUBLISHER_NAME = 'dummy_distance'

def talker():
    rospy.loginfo('Initializing and starting ' + ROS_PUBLISHER_NAME + ' topic.')

    # ROS publisher
    publisher = rospy.Publisher(ROS_PUBLISHER_NAME, Float32, queue_size=10)
    
    # Initialization
    rospy.init_node('talker', anonymous=True)

    # Communication rate
    rate = rospy.Rate(60) # 60 Hz = 30 FPS

    rospy.loginfo(ROS_PUBLISHER_NAME + ' topic has been initialized.')

    while not rospy.is_shutdown():
        distance = randint(0, 9)
        rospy.loginfo(distance)
        publisher.publish(distance)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass