#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes

'''
Created on 10.10.2016

@author: Alberto Romay
'''

class JointStateToMoveit(EventState):
        '''
        State to send a joint state configuration to MoveIt to plan and move.

        ># config_name          string              Name of the joint configuration of interest.

        ># move_group           string              Name of the move group to be used for planning.

        ># action_topic         string              Topic on which MoveIt is listening for action calls.

        ># robot_name           string              Optional name of the robot to be used.
                                                                If left empty, the first one found will be used
                                                                (only required if multiple robots are specified in the same file).

        ># joint_names          string[]            Names of the target joints.
                                                                        Same order as their corresponding names in joint_values.

        ># joint_values         float[]             Target configuration of the joints.
                                                                        Same order as their corresponding names in joint_names.

        <= reached                                  Target joint configuration has been reached.
        <= planning_failed                          Failed to find a plan to the given joint configuration.
        <= control_failed                           Failed to move the arm along the planned trajectory.

        '''

        def __init__(self):
                '''
                Constructor
                '''
                super(JointStateToMoveit, self).__init__(input_keys=['config_name', 'move_group', 'robot_name', 'action_topic', 'joint_values', 'joint_names'],
                                                        outcomes=['reached', 'planning_failed', 'control_failed'],
                                                        output_keys=['config_name', 'move_group', 'robot_name',  'action_topic', 'joint_values', 'joint_names'])

        def execute(self, userdata):
                '''
                Execute this state
                '''
                if self._planning_failed:
                        return 'planning_failed'
                if self._control_failed:
                        return 'control_failed'
                if self._success:
                        return 'reached'

                if self._client.has_result(self._action_topic):
                        result = self._client.get_result(self._action_topic)

                        if result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
                                Logger.logwarn('Control failed for move action of group: %s (error code: %s)' % (self._move_group, str(result.error_code)))
                                self._control_failed = True
                                return 'control_failed'
                        elif result.error_code.val != MoveItErrorCodes.SUCCESS:
                                Logger.logwarn('Move action failed with result error code: %s' % str(result.error_code))
                                self._planning_failed = True
                                return 'planning_failed'
                        else:
                                self._success = True
                                return 'reached'

        def on_enter(self, userdata):
                self._planning_failed = False
                self._control_failed  = False
                self._success         = False
                self._config_name     = userdata.config_name  # Currently not used
                self._robot_name      = userdata.robot_name  # Currently not used
                self._move_group      = userdata.move_group
                self._action_topic    = userdata.action_topic
                self._joint_config    = userdata.joint_values
                self._joint_names     = userdata.joint_names

                self._client = ProxyActionClient({self._action_topic: MoveGroupAction})

                # Action Initialization
                action_goal = MoveGroupGoal()
                action_goal.request.group_name = self._move_group
                action_goal.request.allowed_planning_time = 1.0
                goal_constraints = Constraints()
                for i in range(len(self._joint_names)):
                        goal_constraints.joint_constraints.append(JointConstraint(
                                joint_name=self._joint_names[i],
                                position=self._joint_config[i],
                                weight=1.0))
                action_goal.request.goal_constraints.append(goal_constraints)

                try:
                        self._client.send_goal(self._action_topic, action_goal)
                        userdata.action_topic = self._action_topic  # Save action topic to output key
                except Exception as e:
                        Logger.logwarn('Failed to send action goal for group: %s\n%s' % (self._move_group, str(e)))
                        self._planning_failed = True

        def on_stop(self):
                try:
                        if self._client.is_available(self._action_topic) \
                        and not self._client.has_result(self._action_topic):
                                self._client.cancel(self._action_topic)
                except:
                        # client already closed
                        pass

        def on_pause(self):
                self._client.cancel(self._action_topic)

        def on_resume(self, userdata):
                self.on_enter(userdata)
