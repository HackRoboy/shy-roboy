#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, Int8
import time

# This script listens shy_roboy/state
# and if state is SHOUT or WATCH_PERSON
# it runs /roboy/control/matrix/leds/mode/simple
# with mode=3 parameter to make
# LEDs blink red and blue, like a police car.

class LedReaction:
    def __init__(self):
        self.last_publish_time = None
        self.previous_state = None
        self.red_and_blue_led = Int32(3)
        self.shut_down_led = Int32(0)
        self.sleep_time_after_execution = 6

        # Initialize node and start listening shy_roboy/state
        rospy.loginfo('Activating LED listener.')
        rospy.init_node('led_listener', anonymous=True)
        rospy.Subscriber('shy_roboy/state', Int8, self.led_service_callback)
        rospy.loginfo('Successfully activated LED listener.')

        # Initialize publisher to change LEDs.
        rospy.loginfo('Activating LED publisher.')
        self.publisher = rospy.Publisher('/roboy/control/matrix/leds/mode/simple', Int32, queue_size=10)
        rospy.loginfo('Successfully activated LED publisher.')

    def led_service_callback(self, data):
        # If state is SHOUT or WATCH_PERSON
        if data.data == 2 or data.data == 3:
            if self.last_publish_time is None:
                rospy.loginfo('Sending change LED command.')
                self.previous_state = data.data
                self.last_publish_time = time.time()
                self.publisher.publish(self.red_and_blue_led)
            # and if we waited a while after last publish,
            # make LEDs blink red and blue.
            elif (time.time() - last_publish_time) > self.sleep_time_after_execution:
                rospy.loginfo('Sending change LED command.
                self.previous_state = data.data
                self.last_publish_time = time.time()
                self.publisher.publish(self.red_and_blue_led)
        # If state is IDLE or PERSON_OCCURED
        elif data.data == 1 or data.data == 4:
            if self.previous_state is None:
                self.previous_state = 1
            # and if we used LEDs previously,
            # turn them off.
            elif previous_state == 2 or previous_state == 3:
                rospy.loginfo('Sending turn off LED command.')
                self.previous_state = data.data
                self.publisher.publish(self.shut_down_led)

if __name__ == "__main__":
    try:
        lr = LedReaction()
        rospy.spin()
    except Exception as e:
        print e