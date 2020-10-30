#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import JointState

'''
Created on 10.11.2016

@author: Alberto Romay
'''

class GetJointValuesDynState(EventState):
	'''
	Retrieves current values of specified joints.

	># joint_names	string[]	List of desired joint names.

	#> joint_values float[] 	List of current joint values.

	<= retrieved 				Joint values are available.

	'''

	def __init__(self):
		'''
		Constructor
		'''
		super(GetJointValuesDynState, self).__init__(
			outcomes=['retrieved'],
			output_keys=['joint_values'],
			input_keys=['joint_names'])
		
		self._topic = '/joint_states'
		self._sub = ProxySubscriberCached({self._topic: JointState})

		self._joints = None
		self._joint_values = list()
			
		
	def execute(self, userdata):
		while self._sub.has_buffered(self._topic):
			msg = self._sub.get_from_buffer(self._topic)
			for i in range(len(msg.name)):
				if msg.name[i] in self._joints \
				and self._joint_values[self._joints.index(msg.name[i])] is None:
					self._joint_values[self._joints.index(msg.name[i])] = msg.position[i]

		if all(v is not None for v in self._joint_values):
			userdata.joint_values = self._joint_values
			return 'retrieved'

			
	def on_enter(self, userdata):
		self._sub.enable_buffer(self._topic)
		self._joint_values = [None] * len(self._joints)
		self._joints = userdata.joint_names


	def on_exit(self, userdata):
		self._sub.disable_buffer(self._topic)
