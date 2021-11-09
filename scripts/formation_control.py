#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Formation_control():

    def __init__(self):
        rospy.init_node('formation_control1', anonymous=True)
        print('INIT NODE')

        twist_pub1 = rospy.Publisher('/omnid1/cmd_vel', Twist, queue_size=10)

        self.initialize_subscribers()

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():

            rate.sleep()

    def initialize_subscribers(self):
        rospy.Subscriber('/omnid1/odom', Odometry, self.omnid1_pose)

    def omnid1_pose(self, msg):
        print(msg.pose.pose)






if __name__ == '__main__':
    try:
        Formation_control()
    except rospy.ROSInterruptException:
        pass
