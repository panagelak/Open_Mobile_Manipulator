#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from lynxbot_flexbe_states.drive_forward import GoForwardState
from lynxbot_flexbe_states.turn_state import TurnState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jan 22 2020
@author: Panagiotis
'''
class TurnSM(Behavior):
	'''
	robot turns
	'''


	def __init__(self):
		super(TurnSM, self).__init__()
		self.name = 'Turn'

		# parameters of this behavior
		self.add_parameter('turn_angle', 1.57)
		self.add_parameter('t_speed', 0.4)
		self.add_parameter('speed', 0.4)
		self.add_parameter('travel_dist', 0.5)
		self.add_parameter('obstacle_dist', 0.2)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:774 y:172, x:754 y:311
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Drive Forward State',
										GoForwardState(speed=self.speed, travel_dist=self.travel_dist, obstacle_dist=self.obstacle_dist),
										transitions={'failed': 'Turn State', 'done': 'Turn State'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

			# x:403 y:96
			OperatableStateMachine.add('Turn State',
										TurnState(turn_angle=self.turn_angle, t_speed=self.t_speed),
										transitions={'done': 'finished', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
