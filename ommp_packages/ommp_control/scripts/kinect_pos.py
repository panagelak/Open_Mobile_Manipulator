#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def kinect_mover_pub():
    pub = rospy.Publisher('/kinect_controller/command', Float64, queue_size=10)
    rospy.init_node('kinect_mover', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():

        pub.publish(0.64) #0.66 for not seeing itself with arm in out_of_view position
        rate.sleep()

if __name__ == '__main__':
    try:
        kinect_mover_pub()
    except rospy.ROSInterruptException:
        pass

