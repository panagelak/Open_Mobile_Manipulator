#!/usr/bin/env python

import rospy
import os
import xml.etree.ElementTree as ET
from rospkg import RosPack
from flexbe_core import EventState, Logger

'''
Created on 18.06.2016

@author: Alberto Romay
'''

class GetJointsFromSrdfGroup(EventState):
	'''
	Simple state to look up a pre-defined joint configuration from the given joint group in a SRDF file.
	This state is recommended if you only need these values without any unnecessary overhead.

	-- move_group 	string 		Name of the move group of interest.
								e.g., "my_moveit_config/config/my_robot.srdf"
	-- robot_name 	string 		Optional name of the robot to be used.
								If left empty, the first one found will be used
								(only required if multiple robots are specified in the same file).

	#> joint_names string[] 	List of joint values for the requested group.

	<= retrieved 				Joint values are available.
	<= param_error 				Something went wrong when accessing the SRDF file.

	'''

	def __init__(self, move_group, robot_name=""):
		'''
		Constructor
		'''
		super(GetJointsFromSrdfGroup, self).__init__(outcomes=['retrieved', 'param_error'],
												output_keys=['joint_names'])

		self._move_group = move_group
		self._robot_name = robot_name

		# Check existence of SRDF parameter.
		# Anyways, values will only be read during runtime to allow modifications.
		self._srdf_param = None
		if rospy.has_param("/robot_description_semantic"):
			self._srdf_param = rospy.get_param("/robot_description_semantic")
		else:
			Logger.logerr('Unable to get parameter: /robot_description_semantic')

		self._file_error = False
		self._srdf = None

		
	def execute(self, userdata):


		robot = None
		for r in self._srdf.iter('robot'):
			if self._robot_name == '' or self._robot_name == r.attrib['name']:
				robot = r
				break
		if robot is None:
			Logger.logwarn('Did not find robot name in SRDF: %s' % self._robot_name)
			return 'param_error'

		group = None
		for g in robot.iter('group'):
			if g.attrib['name'] == self._move_group:
				group = g
				break
		if group is None:
			Logger.logwarn('Did not find group name in SRDF: %s' % self._move_group)
			return 'param_error'

		try:
			userdata.joint_names = [str(j.attrib['name']) for j in group.iter('joint')]
		except Exception as e:
			Logger.logwarn('Unable to parse joint values from SRDF:\n%s' % str(e))
			return 'param_error'

		return 'retrieved'

			
	def on_enter(self, userdata):
		# Parameter check
		if self._srdf_param is None:
			self._param_error = True
			return

		try:
			self._srdf = ET.fromstring(self._srdf_param)
		except Exception as e:
			Logger.logwarn('Unable to parse given SRDF parameter: /robot_description_semantic')
			self._param_error = True