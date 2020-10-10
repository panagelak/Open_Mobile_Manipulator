#!/usr/bin/env python

import rospy
import os
import xml.etree.ElementTree as ET
from rospkg import RosPack
from flexbe_core import EventState, Logger

'''
Created on 18.06.2016

@author: Philipp Schillinger
'''

class GetJointsFromSrdfState(EventState):
	'''
	Simple state to look up a pre-defined joint configuration from the given SRDF file.
	This state is recommended if you only need these values without any unnecessary overhead.

	-- config_name 	string 		Name of the joint configuration of interest. 	
	-- srdf_file	string		Package-relative path to the SRDF file,
								e.g., "my_moveit_config/config/my_robot.srdf"	
	-- move_group 	string 		Optional move group name to which the config refers.
								If left empty, the first one found will be used
								(only required if several groups use the same configs).
	-- robot_name 	string 		Optional name of the robot to be used.
								If left empty, the first one found will be used
								(only required if multiple robots are specified in the same file).

	#> joint_values float[] 	List of joint values for the requested config.

	<= retrieved 				Joint values are available.
	<= file_error 				Something went wrong when accessing the SRDF file.

	'''

	def __init__(self, config_name, srdf_file, move_group = "", robot_name = ""):
		'''
		Constructor
		'''
		super(GetJointsFromSrdfState, self).__init__(outcomes=['retrieved', 'file_error'],
												output_keys=['joint_values'])

		self._config_name = config_name
		self._move_group = move_group
		self._robot_name = robot_name

		# Check existence of SRDF file to reduce risk of runtime failure.
		# Anyways, values will only be read during runtime to allow modifications.
		self._srdf_path = None
		try:
			rp = RosPack()
			pkg_name = srdf_file.split('/')[0]
			self._srdf_path = os.path.join(rp.get_path(pkg_name), '/'.join(srdf_file.split('/')[1:]))
			if not os.path.isfile(self._srdf_path):
				raise IOError('File "%s" does not exist!' % self._srdf_path)
		except Exception as e:
			Logger.logwarn('Unable to find given SRDF file: %s\n%s' % (srdf_file, str(e)))

		self._file_error = False
		self._srdf = None

		
	def execute(self, userdata):
		if self._file_error:
			return 'file_error'
		found_other = False

		robot = None
		for r in self._srdf.iter('robot'):
			if self._robot_name == '' or self._robot_name == r.attrib['name']:
				robot = r
				break
		if robot is None:
			Logger.logwarn('Did not find robot name in SRDF: %s' % self._robot_name)
			return 'file_error'

		config = None
		for c in robot.iter('group_state'):
			if (self._move_group == '' or self._move_group == c.attrib['group']) \
			and c.attrib['name'] == self._config_name:
				config = c
				break
		if config is None:
			Logger.logwarn('Did not find config name in SRDF: %s' % self._config_name)
			return 'file_error'

		try:
			userdata.joint_values = [float(j.attrib['value']) for j in config.iter('joint')]
		except Exception as e:
			Logger.logwarn('Unable to parse joint values from SRDF:\n%s' % str(e))
			return 'file_error'

		return 'retrieved'

			
	def on_enter(self, userdata):
		self._file_error = False
		if self._srdf_path is None:
			self._file_error = True
			return

		try:
			self._srdf = ET.parse(self._srdf_path).getroot()
		except Exception as e:
			Logger.logwarn('Unable to parse given SRDF file: %s\n%s' % (self._srdf_path, str(e)))
			self._file_error = True
