#!/usr/bin/env python

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# the list of points to patrol
waypoints = [
    ['one', (6.35, 0.8, 0.1)],
    ['two', (-1.9, -0.17, 0.1)],
    ['three',(3.44,1.05,0.1)]]
class Patrol:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def set_goal_to_point(self, point):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, point[2])
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()


if __name__ == '__main__':
    rospy.init_node('patrolling')
    try:
        p = Patrol()
        while not rospy.is_shutdown():
            for i, w in enumerate(waypoints):
                rospy.loginfo("Sending waypoint %d - %s", i, w[0])
                p.set_goal_to_point(w[1])
    except rospy.ROSInterruptException:
        rospy.logerr("Something went wrong when sending the waypoints")
