import rospy
from std_msgs.msg import Int32, Int8


ROS_PUBLISHER_NAME = '/roboy/control/matrix/leds/mode/simple'
RED_AND_BLUE_LED = Int32(3)

def callback(data):
    if data.data == 2 or data.data == 3:
        rospy.loginfo('Initializing and starting ' + ROS_PUBLISHER_NAME + ' topic.')
        
        # ROS publisher
        publisher = rospy.Publisher(ROS_PUBLISHER_NAME, Int32, queue_size=10)
        
        # Initialization
        rospy.init_node('led_activate_led', anonymous=True)

        rospy.loginfo(ROS_PUBLISHER_NAME + ' topic has been initialized.')

        rospy.loginfo('Sending change LED command.')
        publisher.publish(RED_AND_BLUE_LED)

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