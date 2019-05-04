#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from enum import Enum
import time

threshold = 1.0
delay_occur = 1.0
delay_shout_again = 5.0

class States(Enum):
	IDLE = 0
	PERSON_OCCURED = 1
	SHOUT = 2
	WATCH_PERSON = 3


class TellPeopleToLeave(object):
	def __init__(self):
		self._state = States.IDLE
		self._t = time.time()

	def process_distance_measure(self, data):
		distance = data.data

		newstate = self._state

		if self._state == States.IDLE:
			if distance <= threshold:
				newstate = States.PERSON_OCCURED
				self._t = time.time()
		elif self._state == States.PERSON_OCCURED:
			if distance > threshold:
				newstate = States.IDLE
			elif time.time() - self._t >= delay_occur:
				newstate = States.SHOUT
		elif self._state == States.SHOUT:
			newstate = States.WATCH_PERSON
			self._t = time.time()
		elif self._state == States.WATCH_PERSON:
			if distance > threshold:
				newstate = States.IDLE
			elif time.time() - self._t >= delay_shout_again:
				newstate = SHOUT

		if newstate != self._state:
			shout_node.getlogger().info('New state: {}'.format(newstate))

		if newstate == States.SHOUT:
			self.shout()

		self._state = newstate


	def shout(self):
		shout_node.getlogger().info('\tSHOUT')
		        

if __name__ == '__main__':
    tptl = TellPeopleToLeave()
    rclpy.init()
    global shout_node = rclpy.create_node('tell_people_to_leave')
    shout_node.create_subscription(Float32, 'shy_roboy/nearest_distance',  self.process_distance_measure)
    while rclpy.ok()
        rclpy.spin_once(shout_node)
    shout_node.destroy_node()
    rclpy.shutdown()

