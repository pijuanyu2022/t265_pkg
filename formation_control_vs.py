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
        zd = self.get_distance()
        theta_1d = self.get_omnid_pose('omnid1_posi_z')
        theta_2d = self.get_omnid_pose('omnid2_posi_z')
        theta_3d = self.get_omnid_pose('omnid3_posi_z') 

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():  
            
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

            # get pose of three robots
            pos1 = np.array([[x_1], [y_1], [theta_1]])
            pos2 = np.array([[x_2], [y_2], [theta_2]])
            pos3 = np.array([[x_2], [y_3], [theta_3]])

            # publish twist1 msg
            twist1_msg = Twist()
            twist1_msg.linear.x = self.get_omnid_pose('group_cmd_x') 
            twist1_msg.linear.y = self.get_omnid_pose('group_cmd_y') 
            twist1_msg.linear.y = self.get_omnid_pose('group_cmd_y')

            twist_pub1.publish(twist1_msg)

            #publish twist2 msg
            twist2_msg = Twist()
            twist2_msg.linear.x = self.get_omnid_pose('group_cmd_x') 
            twist2_msg.linear.y = self.get_omnid_pose('group_cmd_y') 
            twist2_msg.linear.y = self.get_omnid_pose('group_cmd_y')

            twist_pub2.publish(twist2_msg)

            #publish twist3 msg
            twist3_msg = Twist()
            twist3_msg.linear.x = self.get_omnid_pose('group_cmd_x') 
            twist3_msg.linear.y = self.get_omnid_pose('group_cmd_y') 
            twist3_msg.linear.y = self.get_omnid_pose('group_cmd_y')

            twist_pub3.publish(twist3_msg)      


            # translation
            if self.get_omnid_pose('group_cmd_x') > 0:

                twist2_msg.linear.x = self.get_omnid_pose('group_cmd_x')*np.cos(theta_2d)
                twist2_msg.linear.y = -self.get_omnid_pose('group_cmd_x')*np.sin(theta_2d)
                twist_pub2.publish(twist2_msg)

                twist3_msg.linear.x = self.get_omnid_pose('group_cmd_x')*np.cos(theta_3d)
                twist3_msg.linear.y = -self.get_omnid_pose('group_cmd_x')*np.sin(theta_3d)
                twist_pub3.publish(twist3_msg)

            if self.get_omnid_pose('group_cmd_x') < 0:

                twist2_msg.linear.x = self.get_omnid_pose('group_cmd_x')*np.cos(theta_2d)
                twist2_msg.linear.y = -self.get_omnid_pose('group_cmd_x')*np.sin(theta_2d)
                twist_pub2.publish(twist2_msg)
                
                twist3_msg.linear.x = self.get_omnid_pose('group_cmd_x')*np.cos(theta_3d)
                twist3_msg.linear.y = -self.get_omnid_pose('group_cmd_x')*np.sin(theta_3d)
                twist_pub3.publish(twist3_msg)
            
            # rotation
            if self.get_omnid_pose('group_cmd_z') > 0:
                twist1_msg.linear.x = -zd[0]*np.sin(theta_1d)
                twist1_msg.linear.y = zd[0]*np.cos(theta_1d)                                                                                                                                                                                        
                twist1_msg.angular.z = self.get_omnid_pose('group_cmd_z')

                twist2_msg.linear.x = zd[2]*np.cos(theta_2d)
                twist2_msg.linear.y = -zd[2]*np.sin(theta_2d)
                twist2_msg.angular.z = self.get_omnid_pose('group_cmd_z')

                twist3_msg.linear.x = -zd[2]*np.cos(theta_3d)
                twist3_msg.linear.y = zd[2]*np.sin(theta_3d)
                twist3_msg.angular.z = self.get_omnid_pose('group_cmd_z')
                
                twist_pub1.publish(twist1_msg)
                twist_pub3.publish(twist3_msg)
                twist_pub2.publish(twist2_msg)

            
            if self.get_omnid_pose('group_cmd_z') < 0:
                twist1_msg.linear.x = zd[0]*np.sin(theta_1d)
                twist1_msg.linear.y = -zd[0]*np.cos(theta_1d)
                twist1_msg.angular.z = self.get_omnid_pose('group_cmd_z')

                twist2_msg.linear.x = -zd[2]*np.cos(theta_2d)
                twist2_msg.linear.y = zd[2]*np.sin(theta_2d)
                twist2_msg.angular.z = self.get_omnid_pose('group_cmd_z')

                twist3_msg.linear.x = zd[2]*np.cos(theta_3d)
                twist3_msg.linear.y = -zd[2]*np.sin(theta_3d)
                twist3_msg.angular.z = self.get_omnid_pose('group_cmd_z')

                twist_pub1.publish(twist1_msg)
                twist_pub3.publish(twist3_msg)
                twist_pub2.publish(twist2_msg)
            
            # get error 
            z = self.get_distance()
            error1 = z[0] - zd[0]
            error2 = z[1] - zd[1]
            error3 = z[2] - zd[2]
            error = np.array([error1, error2, error3])

            # use PID control to deal with error

            print('---------------------------------------------')
            print('the position of omnid 1 is :')
            print(pos1)
            print('the position of omnid 2 is :')
            print(pos2)
            print('the position of omnid 3 is :')
            print(pos3)
            print('the error arrary is')
            print(error)
            print('---------------------------------------------')

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

        # virtual point 
        vp_x = x_2
        vp_y = y_1
        d_1v = math.sqrt(math.pow((x_1 - vp_x), 2) + math.pow((y_1 - vp_y), 2))
        d_2v = math.sqrt(math.pow((x_2 - vp_x), 2) + math.pow((y_2 - vp_y), 2))
        d_3v = math.sqrt(math.pow((x_3 - vp_x), 2) + math.pow((y_3 - vp_y), 2))
        z = np.array([d_1v, d_2v, d_3v])

        return z






if __name__ == '__main__':
    q = Formation_control()
    rospy.spin()
