#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class GoForwardState_input(EventState):

    def __init__(self, speed, travel_dist, obstacle_dist):
        super(GoForwardState_input, self).__init__(outcomes=['failed', 'done'],
            input_keys=['remaining_travel_dist_IN'],
            output_keys=['remaining_travel_dist_OUT'])
        self._start_time = None
        self.distance_traveled = 0.0
        self.data = None
        self._speed = speed
        self._travel_dist = travel_dist
        self._obstacle_dist = obstacle_dist

        self.vel_topic = 'diff_velocity_controller/cmd_vel'
        self.scan_topic = '/scan'

        #create publisher passing it the vel_topic_name and msg_type
        self.pub = ProxyPublisher({self.vel_topic: Twist})
        #create subsciber
        self.scan_sub = ProxySubscriberCached({self.scan_topic: LaserScan})
        #self.scan_sub.set_callback(self.scan_topic, self.scan_callback)

    def execute(self, userdata):
        if not self.cmd_pub:
            return 'failed'
        #run obstacle checks [index 0: Left, 360: middle, 719:right]
        if(self.scan_sub.has_msg(self.scan_topic)):
            self.data = self.scan_sub.get_last_msg(self.scan_topic)
            self.scan_sub.remove_last_msg(self.scan_topic)
            Logger.loginfo('FWD obstacle distance is: %s' % self.data.ranges[360])
            if self.data.ranges[360] <= self._obstacle_dist:
                self.data = None
                userdata.remaining_travel_dist_OUT = self._travel_dist - self.distance_traveled
                return 'failed'

            #measure distance traveled
            elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
            self.distance_traveled = elapsed_time * self._speed

            if self.distance_traveled >= self._travel_dist:
                return 'done'

            #drive
            self.pub.publish(self.vel_topic, self.cmd_pub)

    def on_enter(self, userdata):
        Logger.loginfo('Drive FWD STARTED!')
        self._start_time = rospy.Time.now()
        #set robot speed here
        self.cmd_pub = Twist()
        self.cmd_pub.linear.x = self._speed
        self.cmd_pub.angular.z = 0.0

        if userdata.remaining_travel_dist_IN:
            self._travel_dist = userdata.remaining_travel_dist_IN
            Logger.loginfo("Remaing Distance to Travel %s" % self._travel_dist)

    def on_exit(self, userdata):
        self.cmd_pub.linear.x = 0.0
        self.pub.publish(self.vel_topic, self.cmd_pub)
        Logger.loginfo('Drive FWD ENDED!')

    def on_start(self):
        Logger.loginfo('Drive FWD READY!')

    def on_stop(self):
        Logger.loginfo('Drive FWD STOPPED!')

    def scan_callback(self, data):
        self.data = data




