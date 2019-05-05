#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from enum import Enum
import time

# This script listens shy_roboy/nearest_distance
# and with using state machine that is in the README
# it publishes state of the program via shy_roboy/state

# Distance threshold in meters.
threshold = 1.0

# If person is closer than threshold for this amount of time (in seconds),
# we count it as a personal space violation.
delay_occur = 1.0

# If Roboy shouted to a person, yet he did not leave for this amount of time,
# shout again (in seconds).
delay_shout_again = 5.0

# States that we publish.
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

		# If state is IDLE
		if self._state == States.IDLE:
			# and if something is close,
			if distance <= threshold:
				# Switch to PERSON_OCCURED state and save the time.
				newstate = States.PERSON_OCCURED
				self._t = time.time()
		# If state is PERSON_OCCURED
		elif self._state == States.PERSON_OCCURED:
			# and if object left the personal space
			if distance > threshold:
				# switch to IDLE state.
				newstate = States.IDLE
			# or if object did not leave the personal space
			# for a while
			elif time.time() - self._t >= delay_occur:
				# Switch state to SHOUT
				newstate = States.SHOUT
		# If state is SHOUT
		elif self._state == States.SHOUT:
			# Switch to WATCH_PERSON state and save shouting time.
			newstate = States.WATCH_PERSON
			self._t = time.time()
		# If state is WATCH_PERSON
		elif self._state == States.WATCH_PERSON:
			# and an object left the personal space,
			if distance > threshold:
				# Switch to IDLE state.
				newstate = States.IDLE
			# or it did not leave the personal space for a while,
			elif time.time() - self._t >= delay_shout_again:
				# Switch to SHOUT state.
				newstate = States.SHOUT

		if newstate != self._state:
			rospy.loginfo('New state: {}'.format(newstate))

		self._publisher.publish(Int8(self._state.value))
		self._state = newstate

	def start_listener(self):
		# Initialize listener to shy_roboy/nearest_distance
		# and publisher to shy_roboy/state
		
		rospy.init_node('shy_states')
		self._publisher = rospy.Publisher('shy_roboy/state', Int8, queue_size=10)
		self._publisher.publish(Int8(self._state.value))
		rospy.Subscriber('shy_roboy/nearest_distance', Float32, self.process_distance_measure)
		

if __name__ == '__main__':
	thething = DistanceFiltering()
	thething.start_listener()
	rospy.spin()
