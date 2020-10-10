#!/usr/bin/env python
import rospy
import math
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from spin_action_server.msg import SpinAction, SpinGoal, SpinResult, SpinFeedback

class TurnState(EventState):
	'''
	-- turn angle	float	The angle that the robot should make
	-- t_speed	float    speed at which to turn the robot
	'''

	def __init__(self, turn_angle, t_speed):
		super(TurnState, self).__init__(outcomes=['done', 'failed'])
		self._turn_angle = turn_angle
		self._t_speed = t_speed

		self._topic = '/spin_server_X'

		#create the action client passing it the spin_topic_name and msg_type
		self._client = ProxyActionClient({self._topic: SpinAction}) # pass required clients as dict (topic: type)

		# It may happen that the action client fails to send the action goal
		self._error = False

	def execute(self, userdata):
		#While this state is active, check if the action has been finished and evaluate the result.

		#Check if the client failed to send the goal
		if self._error:
			return 'failed'

		#check if the action has been finished
		if self._client.has_result(self._topic):
			result = self._client.get_result(self._topic)
			result_val = result.done

			#Based on the result, decide which outcome to trigger.
			if result_val == 'done':
				return 'done'
			else:
				return 'failed'

		#check if there is any feedback
		if self._client.has_feedback(self._topic):
			feedback = self._client.get_feedback(self._topic)
			Logger.loginfo("Current heading is: %s" % feedback.heading)

		# If the action has not yet finished, no outcome will be returned and the state stays active.

	def on_enter(self, userdata):
		# When entering this state, we send the action goal once to let the robot start its work
		# Create the goal
		goal = SpinGoal()
		goal.angle = self._turn_angle
		goal.turn_speed = self._t_speed

		# Send the goal
		self._error = False # make sure to reset the error state since a previous state execution might have failed
		try:
			self._client.send_goal(self._topic, goal)
		except Exception as e:
			#Since a state failure not necessarily causes a behavior failure, it is recommended to only print warning, not errors
			# Usinf a linebreak before appending the error Log enables the operator to collapse details in the GUI
			Logger.logwarn('Failed to send the Spin command:\n%s' % str(e))
			self._error = True 

	def on_exit(self, userdata):
		# Make sure that the action is not running when leaving this state.
		# A situation where the action would still be active is for example when the operator manually triggers an outcome.

		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			Logger.loginfo('Cancelled active spin action goal.')

	def on_start(self):
		Logger.loginfo('Robot Turn state: READY!')

	def on_stop(self):
		Logger.loginfo('Robot Turn: Disengaged!')

		#same implementation as on exit
		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			Logger.loginfo('Cancelled active spin action goal.')