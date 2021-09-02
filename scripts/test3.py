#!/usr/bin/env python

import rospy
import sys
from robot_control1 import RobotControl

class UR5Control(RobotControl):
    def person_subscriber(self):
        super(UR5Control, self).__init__(robot_type='UR5')
        rospy.init_node('person_subscriber', anonymous=True)
        IR1_X = self.get_coppeliasim_Value('IR1_x')
        print('step4')
        print(IR1_X)

if __name__ == "__main__":
    q = UR5Control()
    rospy.spin()