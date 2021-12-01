#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_control import RobotControl
import numpy as np
from numpy import pi
import math
import tf
from math import acos, degrees, atan2, radians

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

        # get initial distance
        r, x0, y0, theta_1d, theta_2d, theta_3d = self.get_distance()
        theta_1t = self.get_omnid_pose('omnid1_posi_z')
        theta_2t = self.get_omnid_pose('omnid2_posi_z')
        theta_3t = self.get_omnid_pose('omnid3_posi_z') 
        if theta_1t < 0:
            theta_1t = theta_1t + 2*pi
        elif theta_2t < 0:
            theta_2t = theta_2t + 2*pi
        elif theta_3t < 0:
            theta_3t = theta_3t + 2*pi
        
        theta_1t = self.degree_limit(theta_1t)
        theta_2t = self.degree_limit(theta_2t)
        theta_3t = self.degree_limit(theta_3t)
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():  
            
            # get robot 1, robot 2 and robot 3 position and orientation
            x_1 = self.get_omnid_pose('omnid1_posi_x')             # robot_1 x position
            y_1 = self.get_omnid_pose('omnid1_posi_y')             # robot_1 y position
            theta_1 = self.get_omnid_pose('omnid1_posi_z')                                          # robot_1 orientation theta

            x_2 = self.get_omnid_pose('omnid2_posi_x')             # robot_2 x postion
            y_2 = self.get_omnid_pose('omnid2_posi_y')             # robot_2 y position
            theta_2 = self.get_omnid_pose('omnid2_posi_z')                                          # robot_2 orientation theta

            x_3 = self.get_omnid_pose('omnid3_posi_x')             # robot_3 x position
            y_3 = self.get_omnid_pose('omnid3_posi_y')             # robot_3 y position
            theta_3 = self.get_omnid_pose('omnid3_posi_z')                                          # robot_3 orientation theta

            # get pose of three robots
            pos1 = np.array([[x_1], [y_1], [theta_1]])
            pos2 = np.array([[x_2], [y_2], [theta_2]])
            pos3 = np.array([[x_2], [y_3], [theta_3]])

            # publish twist1 msg
            twist1_msg = Twist()
            twist2_msg = Twist()
            twist3_msg = Twist()

            V1 = np.array([[self.get_omnid_pose('group_cmd_x')],         
                           [self.get_omnid_pose('group_cmd_y')],
                           [self.get_omnid_pose('group_cmd_z')]])
            V2 = V1
            V3 = V1
            

            if self.get_omnid_pose('group_cmd_x') != 0 and self.get_omnid_pose('group_cmd_y') == 0:
                print('x translation !')
                # translation
                V = np.array([[self.get_omnid_pose('group_cmd_x'), 0, 0], 
                            [0,-self.get_omnid_pose('group_cmd_x'), 0],
                            [0, 0, self.get_omnid_pose('group_cmd_z')]])
                
                # robot 1
                G1 = np.array([[np.cos(theta_1t)], 
                            [np.sin(theta_1t)],
                            [0]])
                
                V1 = np.dot(V,G1)

                # robot 2
                G2 = np.array([[np.cos(theta_2t)], 
                            [np.sin(theta_2t)],
                            [0]])
                
                V2 = np.dot(V,G2)

                # robot 3
                G3 = np.array([[np.cos(theta_3t)], 
                            [np.sin(theta_3t)],
                            [0]])
                
                V3 = np.dot(V,G3)
            
            if self.get_omnid_pose('group_cmd_y') != 0 and self.get_omnid_pose('group_cmd_x') == 0:
                print('y translation !')
                # translation
                V = np.array([[self.get_omnid_pose('group_cmd_y'), 0, 0], 
                            [0,self.get_omnid_pose('group_cmd_y'), 0],
                            [0, 0, self.get_omnid_pose('group_cmd_z')]])
                
                # robot 1
                G1 = np.array([[np.sin(theta_1t)], 
                            [np.cos(theta_1t)],
                            [0]])
                
                V1 = np.dot(V,G1)

                # robot 2
                G2 = np.array([[np.sin(theta_2t)], 
                            [np.cos(theta_2t)],
                            [0]])
                
                V2 = np.dot(V,G2)

                # robot 3
                G3 = np.array([[np.sin(theta_3t)], 
                            [np.cos(theta_3t)],
                            [0]])
                
                V3 = np.dot(V,G3)
                

            if self.get_omnid_pose('group_cmd_y') != 0 and self.get_omnid_pose('group_cmd_x') != 0:
                print('x and y translation !')
                if self.get_omnid_pose('group_cmd_y') == self.get_omnid_pose('group_cmd_x'):
                    # translation
                    V = np.array([[self.get_omnid_pose('group_cmd_x'), 0, 0], 
                                [0,self.get_omnid_pose('group_cmd_y'), 0],
                                [0, 0, self.get_omnid_pose('group_cmd_z')]])
                    
                    # robot 1
                    G1 = np.array([[np.cos(pi/4-theta_1t)], 
                                [np.sin(pi/4-theta_1t)],
                                [0]])
                    
                    V1 = np.dot(V,G1)

                    # robot 2
                    G2 = np.array([[np.cos(pi/4-theta_2t)], 
                                [np.sin(pi/4-theta_2t)],
                                [0]])
                    
                    V2 = np.dot(V,G2)

                    # robot 3
                    G3 = np.array([[np.cos(pi/4-theta_3t)], 
                                [np.sin(pi/4-theta_3t)],
                                [0]])
                    
                    V3 = np.dot(V,G3)
                    
                
                else:
                    V = np.array([[self.get_omnid_pose('group_cmd_x'), 0, 0], 
                                [0,self.get_omnid_pose('group_cmd_y'), 0],
                                [0, 0, self.get_omnid_pose('group_cmd_z')]])
                    
                    # robot 1
                    G1 = np.array([[np.sin(pi/4-theta_1t)], 
                                [np.cos(pi/4-theta_1t)],
                                [0]])
                    
                    V1 = np.dot(V,G1)

                    # robot 2
                    G2 = np.array([[np.sin(pi/4-theta_2t)], 
                                [np.cos(pi/4-theta_2t)],
                                [0]])
                    
                    V2 = np.dot(V,G2)

                    # robot 3
                    G3 = np.array([[np.sin(pi/4-theta_3t)], 
                                [np.cos(pi/4-theta_3t)],
                                [0]])
                    
                    V3 = np.dot(V,G3)
                    
            
            twist1_msg.linear.x = V1[0]
            twist1_msg.linear.y = V1[1]
            twist1_msg.angular.z = V1[2]
            twist_pub1.publish(twist1_msg)

            twist2_msg.linear.x = V2[0]
            twist2_msg.linear.y = V2[1]
            twist2_msg.angular.z = V2[2]
            twist_pub2.publish(twist2_msg)

            twist3_msg.linear.x = V3[0]
            twist3_msg.linear.y = V3[1]
            twist3_msg.angular.z = V3[2]
            twist_pub3.publish(twist3_msg)

            # rotation
            if self.get_omnid_pose('group_cmd_z') != 0:

                twist1_msg.linear.x = r*np.cos(theta_1d)*self.get_omnid_pose('group_cmd_z')
                twist1_msg.linear.y = -r*np.sin(theta_1d)*self.get_omnid_pose('group_cmd_z')                                                                                                                                                                                        
                twist1_msg.angular.z = self.get_omnid_pose('group_cmd_z')

                twist2_msg.linear.x = r*np.cos(theta_2d)*self.get_omnid_pose('group_cmd_z')
                twist2_msg.linear.y = -r*np.sin(theta_2d)*self.get_omnid_pose('group_cmd_z')
                twist2_msg.angular.z = self.get_omnid_pose('group_cmd_z')

                twist3_msg.linear.x = r*np.cos(theta_3d)*self.get_omnid_pose('group_cmd_z')
                twist3_msg.linear.y = -r*np.sin(theta_3d)*self.get_omnid_pose('group_cmd_z')
                twist3_msg.angular.z = self.get_omnid_pose('group_cmd_z')
                
                twist_pub1.publish(twist1_msg)
                twist_pub2.publish(twist2_msg)
                twist_pub3.publish(twist3_msg)
            
            print('------------------------------------------')
            print(self.get_omnid_pose('group_cmd_x'))
            print(self.get_omnid_pose('group_cmd_y'))
            print(theta_1t)
            print(pi/4-theta_1t)
            print(np.cos(abs(pi/4-theta_1t)))
            print(abs(pi/4-theta_2t))
            print(theta_2t)
            print(np.cos(abs(pi/4-theta_2t)))
            # print(twist1_msg)
            # print(twist2_msg)
            # print(twist3_msg)

            print('------------------------------------------')

            rate.sleep()
    
    def get_distance(self):
        # get robot 1, robot 2 and robot 3 position and orientation
        x_1 = self.get_omnid_pose('omnid1_posi_x')             # robot_1 x position
        y_1 = self.get_omnid_pose('omnid1_posi_y')             # robot_1 y position
        theta_1 = self.get_omnid_pose('omnid1_posi_z')                                          # robot_1 orientation theta

        x_2 = self.get_omnid_pose('omnid2_posi_x')             # robot_2 x position
        y_2 = self.get_omnid_pose('omnid2_posi_y')             # robot_2 y position
        theta_2 = self.get_omnid_pose('omnid2_posi_z')                                          # robot_2 orientation theta

        x_3 = self.get_omnid_pose('omnid3_posi_x')             # robot_3 x position
        y_3 = self.get_omnid_pose('omnid3_posi_y')             # robot_3 y position
        theta_3 = self.get_omnid_pose('omnid3_posi_z')                                          # robot_3 orientation theta
        
        x0, y0, r = self.circle(x_1, y_1, x_2, y_2, x_3, y_3)

        # point 1d bearing
        theta_1d = pi/2 - self.calculate_bearing(x_1, y_1, theta_1, x0, y0)

        # point 3d bearing
        theta_2d = pi/2 - self.calculate_bearing(x_2, y_2, theta_2, x0, y0)

        # point 3d bearing
        theta_3d = pi/2 - self.calculate_bearing(x_3, y_3, theta_3, x0, y0)

        return r, x0, y0, theta_1d, theta_2d, theta_3d

    def circle(self, x1, y1, x2, y2, x3, y3):
        a = x1 - x2
        b = y1 - y2
        c = x1 - x3
        d = y1 - y3

        a1 = ((x1*x1 - x2*x2) + (y1*y1 - y2*y2))/2.0
        a2 = ((x1*x1 - x3*x3) + (y1*y1 - y3*y3))/2.0
        theta = b*c - a*d
        if abs(theta) < 1e-7:
            raise RuntimeError('There is no triangle')
        x0 = (b*a2 - d*a1)/theta
        y0 = (c*a1 - a*a2)/theta
        r = np.sqrt(pow((x1-x0), 2) + pow((y1 - y0), 2))
        return x0, y0, r

    def calculate_bearing(self, x_1, y_1, theta_1, x_2, y_2):
        dx = x_2 - x_1
        dy = y_2 - y_1
        bearing = atan2(dy, dx)

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
        bearing = self.degree_limit(bearing)
                
        return bearing
    
    def degree_limit(self, theta):
        if theta > 2*pi:
            theta = theta - 2*pi
        elif theta < -2*pi:
            theta = theta + 2*pi
        return theta



if __name__ == '__main__':
    q = Formation_control()
    rospy.spin()
