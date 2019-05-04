#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int8
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


class DistanceFiltering(object):
	def __init__(self):
		self._state = States.IDLE
		self._t = time.time()
		self._publisher = None

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
				newstate = States.SHOUT

		if newstate != self._state:
			rospy.loginfo('New state: {}'.format(newstate))

		self._publisher.publish(Int8(self._state.value))
		self._state = newstate

	def start_listener(self):
		rospy.init_node('shy_states')
		self._publisher = rospy.Publisher('shy_roboy/state', Int8, queue_size=10)
		self._publisher.publish(Int8(self._state.value))
		rospy.Subscriber('shy_roboy/nearest_distance', Float32, self.process_distance_measure)
		

if __name__ == '__main__':
	thething = DistanceFiltering()
	thething.start_listener()
	rospy.spin()
