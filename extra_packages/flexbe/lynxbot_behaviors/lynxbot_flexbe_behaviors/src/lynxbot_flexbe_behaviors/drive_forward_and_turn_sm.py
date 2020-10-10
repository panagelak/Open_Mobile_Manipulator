#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from lynxbot_flexbe_states.drive_forward_input import GoForwardState_input
from lynxbot_flexbe_states.turn_state_input import TurnState_input
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Feb 23 2020
@author: Panagiotis
'''
class DriveForwardAndTurnSM(Behavior):
	'''
	Drives forward until encountering an obstacle
Then it turn 90 degrees
and continues Forward
Repeat 
Until Total Distance Traveled
	'''


	def __init__(self):
		super(DriveForwardAndTurnSM, self).__init__()
		self.name = 'Drive Forward And Turn'

		# parameters of this behavior
		self.add_parameter('speed', 0.5)
		self.add_parameter('travel_dist', 20)
		self.add_parameter('obstacle_dist', 0.7)
		self.add_parameter('turn_angle', 90)
		self.add_parameter('t_speed', 0.5)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:706 y:50, x:742 y:247
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.data_OUT = self.travel_dist

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:75 y:182
			OperatableStateMachine.add('Go Forward State',
										GoForwardState_input(speed=self.speed, travel_dist=self.travel_dist, obstacle_dist=self.obstacle_dist),
										transitions={'failed': 'Turn State', 'done': 'finished'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'remaining_travel_dist_IN': 'data_OUT', 'remaining_travel_dist_OUT': 'data_IN'})

			# x:464 y:193
			OperatableStateMachine.add('Turn State',
										TurnState_input(turn_angle=self.turn_angle, t_speed=self.t_speed),
										transitions={'done': 'Go Forward State', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'data_IN': 'data_IN', 'data_OUT': 'data_OUT'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
