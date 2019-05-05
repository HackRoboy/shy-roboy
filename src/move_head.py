#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

idle_pos = 0 # in radians
max_pos = -0.3 # in radians
start_threshold = 1.0
axis = '1'


class MoveHead(object):
	def __init__(self):
		# Initialize the listener to shy_roboy/nearest_distance
		# and publisher to sphere_axis1/sphere_axis1/target
		rospy.init_node('move_head')
		rospy.Subscriber('shy_roboy/nearest_distance', Float32, self.process_distance_measure)
		self._head_motion = rospy.Publisher('sphere_axis{0}/sphere_axis{0}/target'.format(axis), Float32, queue_size=1)

	def process_distance_measure(self, data):
		# Calculate the angle in radians.
		angle = max(0, start_threshold - data.data) * max_pos + idle_pos
		self._head_motion.publish(angle)
		rospy.loginfo('Angle: {}'.format(angle))


if __name__ == '__main__':
	mh = MoveHead()
	rospy.spin()