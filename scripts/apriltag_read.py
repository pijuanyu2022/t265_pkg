#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool, Int32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range, JointState
import tf2_ros
from tf2_msgs.msg import TFMessage
import geometry_msgs.msg
import time
import numpy as np
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
import math
from apriltag_ros.msg import AprilTagDetectionArray


class RobotControl(object):
    def __init__(self):
        rospy.init_node('person_subscriber', anonymous=True)    
        print('INIT NODE')
        
        self.initialize_subscribers()
        print('step1')
        
        self.IR1_x = Float32()
        print('step2')

        self.SENSOR_TYPES = ['IR1_x']
        print('step3')
    
    def initialize_subscribers(self):
        rospy.Subscriber("/IR1/tag_detections", AprilTagDetectionArray, self.IR1_Callback)
    
    # def tf_cb(self, msg):
    #     self.robot_pose.position.x = msg.transforms[0].transform.translation.x
    #     self.robot_pose.position.y = msg.transforms[0].transform.translation.y
    #     self.robot_pose.position.z = msg.transforms[0].transform.translation.z
    #     self.robot_pose.orientation.x = msg.transforms[0].transform.rotation.x
    #     self.robot_pose.orientation.y = msg.transforms[0].transform.rotation.y
    #     self.robot_pose.orientation.z = msg.transforms[0].transform.rotation.z
    #     self.robot_pose.orientation.w = msg.transforms[0].transform.rotation.w

    def IR1_Callback(self, data):
        self.IR1_x.data = data.detections[0].pose.pose.pose.position.x

    
    #getters 
    def get_coppeliasim_Value(self, sensor_type='IR1_x'):
        assert sensor_type in self.SENSOR_TYPES
        if sensor_type=='IR1_x':
            return self.IR1_x.data

    

    