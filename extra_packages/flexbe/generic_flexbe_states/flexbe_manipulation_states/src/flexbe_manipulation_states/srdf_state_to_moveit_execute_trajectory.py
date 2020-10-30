#!/usr/bin/env python

import rospy
import xml.etree.ElementTree as ET
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyServiceCaller

from moveit_msgs.msg import MoveItErrorCodes#, ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest
from trajectory_msgs.msg import JointTrajectoryPoint

'''
Created on 10.10.2016

@author: Alberto Romay
'''


class SrdfStateToMoveitExecute(EventState):
    '''
        State to execute with MoveIt! a known trajectory defined in the "/robot_description_semantic" parameter (SRDF file) BEWARE! This state performs no self-/collison planning!

        -- config_name          string              Name of the joint configuration of interest.

        -- move_group           string              Name of the move group to be used for planning.

        -- duration             float               Duration of the execution
                                                            Default to 1 second

        -- action_topic         string              Topic on which MoveIt is listening for action calls.
                                                            Defualt to: /execute_kinematic_path

        -- robot_name           string              Optional name of the robot to be used.
                                                                If left empty, the first one found will be used
                                                                (only required if multiple robots are specified in the same file).

        ># joint_config         float[]             Target configuration of the joints.
                                                                        Same order as their corresponding names in joint_names.

        <= reached                                  Target joint configuration has been reached.
        <= request_failed                           Failed to request the service.
        <= moveit_failed                            Failed to execute the known trajectory.

        '''

    def __init__(self, config_name, move_group="", duration=1.0, wait_for_execution=True, action_topic='/execute_kinematic_path', robot_name=""):
        '''
                Constructor
                '''
        super(SrdfStateToMoveitExecute, self).__init__(
            outcomes=['reached', 'request_failed', 'moveit_failed', 'param_error'])

        self._config_name  = config_name
        self._robot_name   = robot_name
        self._move_group   = move_group
        self._duration     = duration
        self._wait_for_execution = wait_for_execution
        self._action_topic = action_topic
        self._client       = ProxyServiceCaller({self._action_topic: ExecuteKnownTrajectory})

        # self._action_topic = action_topic
        # self._client       =  ProxyActionServer({self._action_topic: ExecuteTrajectoryAction})

        self._request_failed = False
        self._moveit_failed  = False
        self._success        = False

        self._srdf_param = None
        if rospy.has_param("/robot_description_semantic"):
            self._srdf_param = rospy.get_param("/robot_description_semantic")
        else:
            Logger.logerr('Unable to get parameter: /robot_description_semantic')

        self._param_error = False
        self._srdf = None
        self._response = None

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self._param_error:
            return 'param_error'
        if self._request_failed:
            return 'request_failed'
        if self._response.error_code.val != MoveItErrorCodes.SUCCESS:
            Logger.logwarn('Move action failed with result error code: %s' % str(self._response.error_code))
            self._moveit_failed = True
            return 'moveit_failed'
        else:
            Logger.loginfo('Move action succeeded: %s' % str(self._response.error_code))
            self._success = True
            return 'reached'

    def on_enter(self, userdata):
        self._param_error    = False
        self._request_failed = False
        self._moveit_failed  = False
        self._success        = False

        # Parameter check
        if self._srdf_param is None:
            self._param_error = True
            return

        try:
            self._srdf = ET.fromstring(self._srdf_param)
        except Exception as e:
            Logger.logwarn('Unable to parse given SRDF parameter: /robot_description_semantic')
            self._param_error = True

        if not self._param_error:

            robot = None
            for r in self._srdf.iter('robot'):
                if self._robot_name == '' or self._robot_name == r.attrib['name']:
                    robot = r
                    break
            if robot is None:
                Logger.logwarn('Did not find robot name in SRDF: %s' % self._robot_name)
                return 'param_error'

            config = None
            for c in robot.iter('group_state'):
                if (self._move_group == '' or self._move_group == c.attrib['group']) \
                        and c.attrib['name'] == self._config_name:
                    config = c
                    self._move_group = c.attrib['group'] #Set move group name in case it was not defined
                    break
            if config is None:
                Logger.logwarn('Did not find config name in SRDF: %s' % self._config_name)
                return 'param_error'

            try:
                self._joint_config = [float(j.attrib['value']) for j in config.iter('joint')]
                self._joint_names = [str(j.attrib['name']) for j in config.iter('joint')]
            except Exception as e:
                Logger.logwarn('Unable to parse joint values from SRDF:\n%s' % str(e))
                return 'param_error'

            # Action Initialization
            action_goal = ExecuteKnownTrajectoryRequest()  # ExecuteTrajectoryGoal()
            #action_goal.trajectory.joint_trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.3)
            action_goal.trajectory.joint_trajectory.joint_names = self._joint_names
            action_goal.trajectory.joint_trajectory.points = [JointTrajectoryPoint()]
            action_goal.trajectory.joint_trajectory.points[0].time_from_start = rospy.Duration(self._duration)
            action_goal.wait_for_execution = self._wait_for_execution

            action_goal.trajectory.joint_trajectory.points[0].positions = self._joint_config

            try:
                self._response = self._client.call(self._action_topic, action_goal)
                Logger.loginfo("Execute Known Trajectory Service requested: \n" + str(self._action_topic))
            except rospy.ServiceException as exc:
                Logger.logwarn("Execute Known Trajectory Service did not process request: \n" + str(exc))
                self._request_failed = True
                return
