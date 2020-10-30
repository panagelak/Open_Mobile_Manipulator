#!/usr/bin/env python
import rospy
import actionlib
from spin_action_server.msg import SpinAction, SpinGoal, SpinResult, SpinFeedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf import transformations
import math

class TurnRobotAction(object):
	def __init__(self):
		self._as = actionlib.SimpleActionServer("spin_server_X", SpinAction, execute_cb=self.execute_cb, auto_start = False)

		self.odom_data = None
		self.initial_orientation = None
		self.cur_orientation = None
		self._turn = False
		self._turn_angle = None
		self.cmd_pub = Twist()
		self._feedback = SpinFeedback()
		self._result = SpinResult()

		self.sub = rospy.Subscriber("/diff_velocity_controller/odom", Odometry, self.odom_callback)
		self.pub = rospy.Publisher("/diff_velocity_controller/cmd_vel", Twist, queue_size = 10)
		self._as.start()

	def execute_cb(self, goal):
		r = rospy.Rate(30)
		self._turn_angle = math.fabs((goal.angle * math.pi)/ 180)
		rospy.loginfo("Target Robot Heading: %s" % self._turn_angle)

		#set initial orientation
		if not self.initial_orientation:
			self.initial_orientation = self.cur_orientation
			self._turn = True 

		while (self.initial_orientation and self._turn):
			#skip if initial and current orientation is same
			if self.initial_orientation == self.cur_orientation:
				continue
			else:
				#calculate the differences in orientation
				cur_angle = transformations.euler_from_quaternion((self.cur_orientation.x, self.cur_orientation.y,
					self.cur_orientation.z, self.cur_orientation.w))
				start_angle = transformations.euler_from_quaternion((self.initial_orientation.x, self.initial_orientation.y,
					self.initial_orientation.z, self.initial_orientation.w))

				turn_so_far = math.fabs(cur_angle[2] - start_angle[2])
				#rospy.loginfo("Current Robot turn at: %f" % turn so far)
				#send feedback (Current Robot heading)
				self._feedback.heading = turn_so_far
				self._as.publish_feedback(self._feedback)
				rospy.loginfo("Current Robot Heading: %s" % turn_so_far)

				if (turn_so_far >= self._turn_angle):
					#reset some variables if done turning
					self._turn = False
					self.initial_orientation = None
					self._result.done = "done"
					self._as.set_succeeded(self._result)
					rospy.loginfo("Turn succesfully completed!")

				#turn
				if self._turn:
					self.cmd_pub.linear.x = 0.0
					if goal.angle > 0:
						self.cmd_pub.angular.z = goal.turn_speed
					else:
						self.cmd_pub.angular.z = -goal.turn_speed
					self.pub.publish(self.cmd_pub)

				else: # stop the robot before breaking out
					self.cmd_pub.angular.z = 0.0
					self.pub.publish(self.cmd_pub)
					break 

				r.sleep()

	def odom_callback(self, data): #get the odometry data
		#get current orientation
		self.cur_orientation = data.pose.pose.orientation

if __name__ == '__main__':
	rospy.init_node('spin_node')
	rospy.loginfo("Spin Action Server started ...")
	server = TurnRobotAction()
	rospy.spin()

