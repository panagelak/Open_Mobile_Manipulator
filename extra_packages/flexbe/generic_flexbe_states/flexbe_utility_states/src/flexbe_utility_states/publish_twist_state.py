#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy

from flexbe_core.proxy import ProxyPublisher
from geometry_msgs.msg import Twist

"""Created on June. 21, 2017

@author: Alireza Hosseini
"""


class PublishTwistState(EventState):
	"""
	Publishes a velocity command from userdata.

	-- topic 		string 			Topic to which the velocity command will be published.

	># twist		Twist			Velocity command to be published.

	<= done						Velcoity command has been published.

	"""
	
	def __init__(self, topic):
		"""Constructor"""
		super(PublishTwistState, self).__init__(outcomes=['done'],
												input_keys=['twist'])

		self._topic = topic
		self._pub = ProxyPublisher({self._topic: Twist})


	def execute(self, userdata):
		return 'done'
	
	def on_enter(self, userdata):
		self._pub.publish(self._topic, userdata.twist)
