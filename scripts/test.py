#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float32
from apriltag_ros.msg import AprilTagDetectionArray

X = Float32()

def tagCallback(data):
    tag_id = data.detections[0].id
    x_pos = data.detections[0].pose.pose.pose.position.x
    y_pos = data.detections[0].pose.pose.pose.position.y
    print("April tag ID is %d"%tag_id)
    print('the x value is :%.3f'%x_pos)
    print('the y value is :%.3f'%y_pos)
    # X = int(str(tag_id)[1])
    # if X == 3:
    #     print("April tag ID is %d"%tag_id)
    #     print('the x value is :%.3f'%x_pos)
    #     print('the y value is :%.3f'%y_pos)

def person_subscriber():
    rospy.init_node('person_subscriber', anonymous=True)
    rospy.Subscriber("tag_detections", AprilTagDetectionArray, tagCallback)
    rospy.spin()

if __name__ == '__main__':
    person_subscriber()