#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry


class GetData(object):
    def __init__(self):     
        self.initialize_subscribers()

        self.IR1_x = Float32()
        self.IR1_y = Float32()
        self.IR1_z = Float32()

        self.IR2_x = Float32()
        self.IR2_y = Float32()
        self.IR2_z = Float32()

        self.Color_x = Float32()
        self.Color_y = Float32()
        self.Color_z = Float32()

        self.Fish1_x = Float32()
        self.Fish1_y = Float32()
        self.Fish1_z = Float32()

        self.Fish2_x = Float32()
        self.Fish2_y = Float32()
        self.Fish2_z = Float32()

        self.T265_x = Float32()
        self.T265_y = Float32()
        self.T265_z = Float32()

        self.TAG_TYPES = ['IR1_x', 'IR1_y', 'IR1_z', 'IR2_x', 'IR2_y', 'IR2_z', 'Color_x', \
            'Color_y', 'Color_z', 'Fish1_x', 'Fish1_y', 'Fish1_z', 'Fish2_x', 'Fish2_y', 'Fish2_z'\
                , 'T265_x', 'T265_y', 'T265_z']
    
    def initialize_subscribers(self):
        rospy.Subscriber("/IR1/tag_detections", AprilTagDetectionArray, self.IR1_tagCallback_x)
        rospy.Subscriber("/IR1/tag_detections", AprilTagDetectionArray, self.IR1_tagCallback_y)
        rospy.Subscriber("/IR1/tag_detections", AprilTagDetectionArray, self.IR1_tagCallback_z)

        rospy.Subscriber("/IR2/tag_detections", AprilTagDetectionArray, self.IR2_tagCallback_x)
        rospy.Subscriber("/IR2/tag_detections", AprilTagDetectionArray, self.IR2_tagCallback_y)
        rospy.Subscriber("/IR2/tag_detections", AprilTagDetectionArray, self.IR2_tagCallback_z)

        rospy.Subscriber("/Color/tag_detections", AprilTagDetectionArray, self.Color_tagCallback_x)
        rospy.Subscriber("/Color/tag_detections", AprilTagDetectionArray, self.Color_tagCallback_y)
        rospy.Subscriber("/Color/tag_detections", AprilTagDetectionArray, self.Color_tagCallback_z)

        rospy.Subscriber("/fisheye1/tag_detections", AprilTagDetectionArray, self.Fish1_tagCallback_x)
        rospy.Subscriber("/fisheye1/tag_detections", AprilTagDetectionArray, self.Fish1_tagCallback_y)
        rospy.Subscriber("/fisheye1/tag_detections", AprilTagDetectionArray, self.Fish1_tagCallback_z)

        rospy.Subscriber("/fisheye2/tag_detections", AprilTagDetectionArray, self.Fish2_tagCallback_x)
        rospy.Subscriber("/fisheye2/tag_detections", AprilTagDetectionArray, self.Fish2_tagCallback_y)
        rospy.Subscriber("/fisheye2/tag_detections", AprilTagDetectionArray, self.Fish2_tagCallback_z)

        rospy.Subscriber("/camera/odom/sample", Odometry, self.T265_tagCallback_x)
        rospy.Subscriber("/camera/odom/sample", Odometry, self.T265_tagCallback_y)
        rospy.Subscriber("/camera/odom/sample", Odometry, self.T265_tagCallback_z)
    

    def IR1_tagCallback_x (self,data):
        if len(data.detections) > 0:
            self.IR1_x.data = data.detections[0].pose.pose.pose.position.x
        else:
            self.IR1_x.data = -1000

    def IR1_tagCallback_y (self,data):
        if len(data.detections) > 0:
            self.IR1_y.data = data.detections[0].pose.pose.pose.position.y
        else:
            self.IR1_y.data = -1000
    
    def IR1_tagCallback_z (self,data):
        if len(data.detections) > 0:
            self.IR1_z.data = data.detections[0].pose.pose.pose.position.z
        else:
            self.IR1_z.data = -1000

    def IR2_tagCallback_x (self,data):
        if len(data.detections) > 0:
            self.IR2_x.data = data.detections[0].pose.pose.pose.position.x
        else:
            self.IR2_x.data = -1000

    def IR2_tagCallback_y (self,data):
        if len(data.detections) > 0:
            self.IR2_y.data = data.detections[0].pose.pose.pose.position.y
        else:
            self.IR2_y.data = -1000
    
    def IR2_tagCallback_z (self,data):
        if len(data.detections) > 0:
            self.IR2_z.data = data.detections[0].pose.pose.pose.position.z
        else:
            self.IR2_z.data = -1000

    def Color_tagCallback_x (self,data):
        if len(data.detections) > 0:
            self.Color_x.data = data.detections[0].pose.pose.pose.position.x
        else:
            self.Color_x.data = -1000

    def Color_tagCallback_y (self,data):
        if len(data.detections) > 0:
            self.Color_y.data = data.detections[0].pose.pose.pose.position.y
        else:
            self.Color_y.data = -1000
    
    def Color_tagCallback_z (self,data):
        if len(data.detections) > 0:
            self.Color_z.data = data.detections[0].pose.pose.pose.position.z
        else:
            self.Color_z.data = -1000


    def Fish1_tagCallback_x (self,data):
        if len(data.detections) > 0:
            self.Fish1_x.data = data.detections[0].pose.pose.pose.position.x
        else:
            self.Fish1_x.data = -1000

    def Fish1_tagCallback_y (self,data):
        if len(data.detections) > 0:
            self.Fish1_y.data = data.detections[0].pose.pose.pose.position.y
        else:
            self.Fish1_y.data = -1000
    
    def Fish1_tagCallback_z (self,data):
        if len(data.detections) > 0:
            self.Fish1_z.data = data.detections[0].pose.pose.pose.position.z
        else:
            self.Fish1_z.data = -1000

    def Fish2_tagCallback_x (self,data):
        if len(data.detections) > 0:
            self.Fish2_x.data = data.detections[0].pose.pose.pose.position.x
        else:
            self.Fish2_x.data = -1000

    def Fish2_tagCallback_y (self,data):
        if len(data.detections) > 0:
            self.Fish2_y.data = data.detections[0].pose.pose.pose.position.y
        else:
            self.Fish2_y.data = -1000
    
    def Fish2_tagCallback_z (self,data):
        if len(data.detections) > 0:
            self.Fish2_z.data = data.detections[0].pose.pose.pose.position.z
        else:
            self.Fish2_z.data = -1000

    def T265_tagCallback_x (self,data):
        self.T265_x.data = data.pose.pose.position.x

    def T265_tagCallback_y (self,data):
        self.T265_y.data = data.pose.pose.position.y
    
    def T265_tagCallback_z (self,data):
        self.T265_z.data = data.pose.pose.position.z
    
    #getters 
    def get_apriltag_Value(self, tag_type='IR1_x'):
        assert tag_type in self.TAG_TYPES
        if tag_type=='IR1_x':
            return self.IR1_x.data
        elif tag_type=='IR1_y':
            return self.IR1_y.data
        elif tag_type=='IR1_z':
            return self.IR1_z.data
        elif tag_type=='IR2_x':
            return self.IR2_x.data
        elif tag_type=='IR2_y':
            return self.IR2_y.data
        elif tag_type=='IR2_z':
            return self.IR2_z.data
        elif tag_type=='Color_x':
            return self.Color_x.data
        elif tag_type=='Color_y':
            return self.Color_y.data
        elif tag_type=='Color_z':
            return self.Color_z.data
        elif tag_type=='Fish1_x':
            return self.Fish1_x.data
        elif tag_type=='Fish1_y':
            return self.Fish1_y.data
        elif tag_type=='Fish1_z':
            return self.Fish1_z.data
        elif tag_type=='Fish2_x':
            return self.Fish2_x.data
        elif tag_type=='Fish2_y':
            return self.Fish2_y.data
        elif tag_type=='Fish2_z':
            return self.Fish2_z.data
        elif tag_type=='T265_x':
            return self.T265_x.data
        elif tag_type=='T265_y':
            return self.T265_y.data
        elif tag_type=='T265_z':
            return self.T265_z.data

    

    