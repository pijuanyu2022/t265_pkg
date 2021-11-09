#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool, Int32
from robot_control import RobotControl

class Formation_control(RobotControl):

    def __init__(self):
        super(Formation_control, self).__init__()
        rospy.init_node('formation_control1', anonymous=True)
        print('INIT NODE')

        twist_pub1 = rospy.Publisher('/omnid1/cmd_vel', Twist, queue_size=10)

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            A1 = self.get_omnid_pose('omnid1_ori_y')
            print(A1)

            rate.sleep()




if __name__ == '__main__':
    q = Formation_control()
    rospy.spin()
