#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_control import RobotControl
import numpy as np
from numpy import pi
import math
import tf
from tf import transformations
from math import acos, atan2, degrees

class Formation_control(RobotControl):

    def __init__(self):

        # initial ros node
        super(Formation_control, self).__init__()
        rospy.init_node('formation_control1', anonymous=True)
        print('INIT NODE')

        # publish velocity rostopic for three robots
        twist_pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        twist_pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        twist_pub3 = rospy.Publisher('/turtle3/cmd_vel', Twist, queue_size=10)

        rospy.sleep(2.0)

        # Get the initial position and bearing

        # desired distance between two robots
        ld_12 = self.initial_distance()
        wd_12 = self.initial_bearing()

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():

            # publish twist1 msg
            twist1_msg = Twist()
            twist1_msg.linear.x = self.get_omnid_pose('group_cmd_x')
            twist1_msg.linear.y = self.get_omnid_pose('group_cmd_y')
            twist1_msg.angular.z = self.get_omnid_pose('group_cmd_z')
            twist_pub1.publish(twist1_msg)

            # Basic leader-following control

            d = 0.3                                                  # distance between robot center to the front          
            
            # get robot 1 and robot 2 position and orientation
            x_1 = self.get_omnid_pose('omnid1_posi_x')             # robot_1 x position
            y_1 = self.get_omnid_pose('omnid1_posi_y')             # robot_1 y position
            theta_1 = self.get_omnid_pose('omnid1_posi_z')                                          # robot_1 orientation theta

            x_2 = self.get_omnid_pose('omnid2_posi_x')             # robot_2 x position
            y_2 = self.get_omnid_pose('omnid2_posi_y')               # robot_2 y position
            theta_2 = self.get_omnid_pose('omnid2_posi_z')                                          # robot_2 orientation theta


            k1 = 5
            k2 = 5                                                  # P controller gain value
            
            # robot 2 front position
            p2_x = x_2 + d*np.cos(theta_2)
            p2_y = y_2 + d*np.sin(theta_2)

            # current separation between two robots
            l_12 = math.sqrt(math.pow((x_1 - p2_x), 2) + math.pow((y_1 - p2_y), 2)) 
            
            # current bearing between two robots
            w_12 = self.calculate_bearing(x_1, y_1, theta_1, p2_x, p2_y, theta_2)                                                     

            u1 = np.array([[self.get_omnid_pose('group_cmd_x')], [self.get_omnid_pose('group_cmd_z')]])   # robot 1 velocity linear.x, angular.z

            b_12 = theta_1 - theta_2                                 # b_12 is the relative orientation
            r_12 = b_12 + w_12               

            p1 = np.array([[k1*(ld_12 - l_12)],   # p1 is an auxiliary control input
                           [k2*(wd_12 - w_12)]])

            F1 = np.array([[-np.cos(w_12), 0],       
                           [np.sin(w_12)/l_12, -1]])
            
            G1 = np.array([[np.cos(r_12), d*np.sin(r_12)], 
                           [-np.sin(r_12)/l_12, d*np.cos(r_12)/l_12]])

            G1_inv = np.linalg.inv(G1)

            u2 = np.dot(G1_inv, (p1 - np.dot(F1, u1)))  # final velocity output of robot 2
            

            #publish twist2 msg
            twist2_msg = Twist()
            twist2_msg.linear.x = u2[0]
            twist2_msg.angular.z = u2[1]
            twist_pub2.publish(twist2_msg)
            dx = x_1 - p2_x
            dy = y_1 - p2_y


            print('---------------------------------------------')
            print('The x position of omnid 1 is ', x_1)
            print('The y position of omnid 1 is ', y_1)
            print('The orientation of omnid 1 is ', degrees(theta_1))
            print('The x position of omnid 2 is ', x_2)
            print('The y position of omnid 2 is ', y_2)
            print('The orientation of omnid 2 is ', theta_2)

            print('The x position of p2 is ', p2_x)
            print('The y position of p2 is ', p2_y)

            print('The current value of l_12 is ', l_12)

            print('The desired value of l_12 is ', ld_12)

            print('The value of w_12 is', degrees(w_12))
            print('The value of wd_12 is', degrees(wd_12))


            print(p1)
            print(F1)
            print(G1_inv)



            print('The robot 1 velocity is')
            print(u1)

            print('The robot 2 velocity is')
            print(u2)

            print('---------------------------------------------')

            rate.sleep()

    def initial_distance(self):
        # Basic leader-following control

        d = 0.3                                                  # distance between robot center to the front          
        
        # get robot 1 and robot 2 position and orientation
        x_1 = self.get_omnid_pose('omnid1_posi_x')             # robot_1 x position
        y_1 = self.get_omnid_pose('omnid1_posi_y')             # robot_1 y position
        theta_1 = self.get_omnid_pose('omnid1_posi_z')                                          # robot_1 orientation theta

        x_2 = self.get_omnid_pose('omnid2_posi_x')             # robot_2 x position
        y_2 = self.get_omnid_pose('omnid2_posi_y')               # robot_2 y position
        theta_2 = self.get_omnid_pose('omnid2_posi_z')                                          # robot_2 orientation theta
        p2_x = x_2 + d*np.cos(theta_2)
        p2_y = y_2 + d*np.sin(theta_2)

        l_12 = math.sqrt(math.pow((x_1 - p2_x), 2) + math.pow((y_1 - p2_y), 2))
        return l_12

    def initial_bearing(self):

        # Basic leader-following control

        d = 0.3                                                  # distance between robot center to the front          
        
        # get robot 1 and robot 2 position and orientation
        x_1 = self.get_omnid_pose('omnid1_posi_x')             # robot_1 x position
        y_1 = self.get_omnid_pose('omnid1_posi_y')             # robot_1 y position
        theta_1 = self.get_omnid_pose('omnid1_posi_z')                                          # robot_1 orientation theta

        x_2 = self.get_omnid_pose('omnid2_posi_x')             # robot_2 x position
        y_2 = self.get_omnid_pose('omnid2_posi_y')               # robot_2 y position
        theta_2 = self.get_omnid_pose('omnid2_posi_z')                                          # robot_2 orientation theta
        p2_x = x_2 + d*np.cos(theta_2)
        p2_y = y_2 + d*np.sin(theta_2)

        l_12 = math.sqrt(math.pow((x_1 - p2_x), 2) + math.pow((y_1 - p2_y), 2)) 
   
        w_12 = self.calculate_bearing(x_1, y_1, theta_1, p2_x, p2_y, theta_2)  
        return w_12

    def calculate_bearing(self, x_1, y_1, theta_1, x_2, y_2, theta_2):
        dx = x_2 - x_1
        dy = y_2 - y_1
        bearing = atan2(dy, dx)

        if theta_1 < -1.414*2:
            theta_1 = theta_1+2*pi

        if theta_1 < 1.414:
            if bearing < 0 :
                bearing = 2*pi+ atan2(dy, dx)-theta_1
            elif bearing > 0:
                bearing = atan2(dy, dx)-theta_1

        if theta_1 > 1.414:
            if bearing < 0:
                bearing = atan2(dy,dx) +2*pi - theta_1
            if bearing > 0:
                bearing = 2*pi - abs(theta_1 -atan2(dy,dx))
        




        return bearing





if __name__ == '__main__':
    q = Formation_control()
    rospy.spin()