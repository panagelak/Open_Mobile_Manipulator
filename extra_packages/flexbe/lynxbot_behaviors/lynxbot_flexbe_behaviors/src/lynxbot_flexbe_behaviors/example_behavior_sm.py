#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.log_state import LogState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 21 2015
@author: Philipp Schillinger
'''
class ExampleBehaviorSM(Behavior):
	'''
	This is a simple example for a behavior.
	'''


	def __init__(self):
		super(ExampleBehaviorSM, self).__init__()
		self.name = 'Example Behavior'

		# parameters of this behavior
		self.add_parameter('waiting_time', 3)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 172 147 
		# This transition will only be executed if the Autonomy Level is greater than Low during execution, e.g. High



	def create(self):
		log_msg = "Hello World!"
		# x:83 y:390
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:52 y:78
			OperatableStateMachine.add('Print_Message',
										LogState(text=log_msg, severity=Logger.REPORT_HINT),
										transitions={'done': 'Wait_After_Logging'},
										autonomy={'done': Autonomy.Low})

			# x:40 y:228
			OperatableStateMachine.add('Wait_After_Logging',
										WaitState(wait_time=self.waiting_time),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
