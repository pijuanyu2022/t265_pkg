#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class RobotControl(object):
    def __init__(self):
        
        self.initialize_subscribers()

        self.omnid1_posi_x = Float32()
        self.omnid1_posi_y = Float32()
        self.omnid1_posi_z = Float32()

        self.omnid1_ori_x = Float32()
        self.omnid1_ori_y = Float32()
        self.omnid1_ori_z = Float32()
        self.omnid1_ori_w = Float32()

        self.omnid2_posi_x = Float32()
        self.omnid2_posi_y = Float32()
        self.omnid2_posi_z = Float32()

        self.omnid2_ori_x = Float32()
        self.omnid2_ori_y = Float32()
        self.omnid2_ori_z = Float32()
        self.omnid2_ori_w = Float32()
        
        self.omnid3_posi_x = Float32()
        self.omnid3_posi_y = Float32()
        self.omnid3_posi_z = Float32()

        self.omnid3_ori_x = Float32()
        self.omnid3_ori_y = Float32()
        self.omnid3_ori_z = Float32()
        self.omnid3_ori_w = Float32()


        self.POSE_TYPES = ['omnid1_posi_x', 'omnid1_posi_y','omnid1_posi_z', \
        'omnid1_ori_x',  'omnid1_ori_y', 'omnid1_ori_z','omnid1_ori_w', \
        'omnid2_posi_x', 'omnid2_posi_y','omnid2_posi_z', \
        'omnid2_ori_x',  'omnid2_ori_y', 'omnid2_ori_z','omnid2_ori_w', \
        'omnid3_posi_x', 'omnid3_posi_y','omnid3_posi_z', \
        'omnid3_ori_x',  'omnid3_ori_y', 'omnid3_ori_z','omnid3_ori_w']
    
    def initialize_subscribers(self):

        rospy.Subscriber('/omnid1/odom', Odometry, self.omnid1_pose)
        rospy.Subscriber('/omnid2/odom', Odometry, self.omnid2_pose)
        rospy.Subscriber('/omnid3/odom', Odometry, self.omnid3_pose)
    

    #Pose callbacks Same for both robots. 
    def omnid1_pose(self, msg):
        self.omnid1_posi_x.data = msg.pose.pose.position.x
        self.omnid1_posi_y.data = msg.pose.pose.position.y
        self.omnid1_posi_z.data = msg.pose.pose.position.z
        self.omnid1_ori_x.data = msg.pose.pose.orientation.x
        self.omnid1_ori_y.data = msg.pose.pose.orientation.y
        self.omnid1_ori_z.data = msg.pose.pose.orientation.z
        self.omnid1_ori_w.data = msg.pose.pose.orientation.w


    def omnid2_pose(self, msg):
        self.omnid2_posi_x.data = msg.pose.pose.position.x
        self.omnid2_posi_y.data = msg.pose.pose.position.y
        self.omnid2_posi_z.data = msg.pose.pose.position.z
        self.omnid2_ori_x.data = msg.pose.pose.orientation.x
        self.omnid2_ori_y.data = msg.pose.pose.orientation.y
        self.omnid2_ori_z.data = msg.pose.pose.orientation.z
        self.omnid2_ori_w.data = msg.pose.pose.orientation.w

    def omnid3_pose(self, msg):
        self.omnid3_posi_x.data = msg.pose.pose.position.x
        self.omnid3_posi_y.data = msg.pose.pose.position.y
        self.omnid3_posi_z.data = msg.pose.pose.position.z
        self.omnid3_ori_x.data = msg.pose.pose.orientation.x
        self.omnid3_ori_y.data = msg.pose.pose.orientation.y
        self.omnid3_ori_z.data = msg.pose.pose.orientation.z
        self.omnid3_ori_w.data = msg.pose.pose.orientation.w
        

    
    #getters 
    def get_omnid_pose(self, pose_type='omnid1_posi_x'):
        assert pose_type in self.POSE_TYPES
        if pose_type=='omnid1_posi_x':
            return self.omnid1_posi_x.data
        elif pose_type=='omnid1_posi_y':
            return self.omnid1_posi_y.data
        elif pose_type=='omnid1_posi_z':
            return self.omnid1_posi_z.data
        elif pose_type=='omnid1_ori_x':
            return self.omnid1_ori_x.data
        elif pose_type=='omnid1_ori_y':
            return self.omnid1_ori_y.data
        elif pose_type=='omnid1_ori_z':
            return self.omnid1_ori_z.data
        elif pose_type=='omnid1_ori_w':
            return self.omnid1_ori_w.data
        elif pose_type=='omnid2_posi_x':
            return self.omnid2_posi_x.data
        elif pose_type=='omnid2_posi_y':
            return self.omnid2_posi_y.data
        elif pose_type=='omnid2_posi_z':
            return self.omnid2_posi_z.data
        elif pose_type=='omnid2_ori_x':
            return self.omnid2_ori_x.data
        elif pose_type=='omnid2_ori_y':
            return self.omnid2_ori_y.data
        elif pose_type=='omnid2_ori_z':
            return self.omnid2_ori_z.data
        elif pose_type=='omnid2_ori_w':
            return self.omnid2_ori_w.data
        elif pose_type=='omnid3_posi_x':
            return self.omnid3_posi_x.data
        elif pose_type=='omnid3_posi_y':
            return self.omnid3_posi_y.data
        elif pose_type=='omnid3_posi_z':
            return self.omnid3_posi_z.data
        elif pose_type=='omnid3_ori_x':
            return self.omnid3_ori_x.data
        elif pose_type=='omnid3_ori_y':
            return self.omnid3_ori_y.data
        elif pose_type=='omnid3_ori_z':
            return self.omnid3_ori_z.data
        elif pose_type=='omnid3_ori_w':
            return self.omnid3_ori_w.data


    

    