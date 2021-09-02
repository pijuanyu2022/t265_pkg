#!/usr/bin/env python
import rospy
import time
import csv
import numpy as np
import matplotlib.pyplot as plt
from robot_control1 import GetData


class Process_data(GetData):

    def __init__(self):
        super(Process_data, self).__init__()
        rospy.init_node('Apriltag', anonymous=True)
        start = time.time()
        # main control loop
        while not rospy.is_shutdown():
            print("--------------------------------------------------------------")

            print("--------------------------------------------------------------")

            time.sleep(0.1) # change the sleep time to whatever is the appropriate control rate for simulation


    
if __name__ == "__main__":
    q = Process_data()
    rospy.spin()