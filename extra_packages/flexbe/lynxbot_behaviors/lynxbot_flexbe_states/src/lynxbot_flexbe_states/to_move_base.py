#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
#from geometry_msgs.msg import Twist
#from sensor_msgs.msg import LaserScan
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D
class ToMoveBase(EventState):

    def __init__(self, x, y, th):
        super(GoForwardState, self).__init__(outcomes=['failed', 'done'],
            output_keys=['waypoint'])
        self._start_time = None


        self._x = x
        self._y = y
        self._th = th




    def execute(self, userdata):
        userdata.waypoint = Pose2D()
        userdata.waypoint.x = self.waypoint.x
        userdata.waypoint.y = self.waypoint.y
        userdata.waypoint.theta = self.waypoint.theta
        return 'done'
        #measure distance traveled
        elapsed_time = (rospy.Time.now() - self._start_time).to_sec()


    def on_enter(self, userdata):
        Logger.loginfo('To move_base STARTED!')
        
        #set robot speed here
        self.waypoint = Pose2D()
        self.waypoint.x = self._x
        self.waypoint.y = self._y
        self.waypoint.theta = self._th
        self._start_time = rospy.Time.now()

    def on_exit(self, userdata):

        Logger.loginfo('To move_base ENDED!')

    def on_start(self):
        Logger.loginfo('To move_base READY!')

    def on_stop(self):
        Logger.loginfo('To move_base STOPPED!')






