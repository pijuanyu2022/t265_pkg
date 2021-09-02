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
        i = 0
        M = []
        N1 = []
        N2 = []
        N3 = []
        start = time.time()
        # main control loop
        while not rospy.is_shutdown():
            print("--------------------------------------------------------------")
            IR1_X = self.get_apriltag_Value('IR1_x')
            IR1_Y = self.get_apriltag_Value('IR1_y')
            IR1_Z = self.get_apriltag_Value('IR1_z')

            IR2_X = self.get_apriltag_Value('IR2_x')
            IR2_Y = self.get_apriltag_Value('IR2_y')
            IR2_Z = self.get_apriltag_Value('IR2_z')

            Color_X = self.get_apriltag_Value('Color_x')
            Color_Y = self.get_apriltag_Value('Color_y')
            Color_Z = self.get_apriltag_Value('Color_z')

            print("The position of apriltag in IR1 camera is: ")
            print("              x: %f"%IR1_X)
            print("              y: %f"%IR1_Y)
            print("              z: %f"%IR1_Z)


            print("The position of apriltag in IR2 camera is: ")
            print("              x: %f"%IR2_X)
            print("              y: %f"%IR2_Y)
            print("              z: %f"%IR2_Z)


            print("The position of apriltag in Color camera is: ")
            print("              x: %f"%Color_X)
            print("              y: %f"%Color_Y)
            print("              z: %f"%Color_Z)
     
            i = i+1
            A = self.get_apriltag_Value('IR1_x')
            B = self.get_apriltag_Value('IR2_x')
            C = self.get_apriltag_Value('Color_x')
            if i < 201:
                N1.append(A)
                N2.append(B)
                N3.append(C)
            else:
                break
            iii = float(i/10)
            print('The time is %f'%iii)

            print("--------------------------------------------------------------")

            time.sleep(0.1) # change the sleep time to whatever is the appropriate control rate for simulation
        
        
        end = time.time()
        T = end-start
        print('The time is', T)
        k = T/200
        j = float(0)
        while j < T:
            j = j+k
            M.append(j)
    
        # print(N2[0]-N1[0])
        # print(N2[0])
        # print(N3[0]-N1[0])

        N2X =[]
        M2 = M.copy()
        i2 = 0

        for i2 in range(199):
            i2 +=1
            N2_1 = N2[i2]-N2[0]+N1[0]
            N2X.append(N2_1)

        N3X =[]
        M3 = M.copy()
        i3 = 0

        for i3 in range(199):
            i3 +=1
            N3_1 = N3[i3]-N3[0]+N1[0]
            N3X.append(N3_1)
        
        N4X = []
        M4 = M.copy()
        i4 = 0

        for i4 in range(198):
            i4 +=1
            N4_1 = (N1[i4]+N2X[i4]+N3X[i4])/3
            N4X.append(N4_1)

        # print(len(M1))
        if len(N1) > len(M):
            N1.pop()
        if len(M) > len(N1):
            M.pop()
        if len(N2X) > len(M2):
            N2X.pop()
        if len(M2) > len(N2X):
            M2.pop()
        if len(N2X) > len(M2):
            N2X.pop()
        if len(M2) > len(N2X):
            M2.pop()
        if len(N3X) > len(M3):
            N3X.pop()
        if len(M3) > len(N3X):
            M3.pop()
        if len(N3X) > len(M3):
            N3X.pop()
        if len(M3) > len(N3X):
            M3.pop()
        if len(N4X) > len(M4):
            N4X.pop()
        if len(M4) > len(N4X):
            M4.pop()
        if len(N4X) > len(M4):
            N4X.pop()
        if len(M4) > len(N4X):
            M4.pop()
        if len(N4X) > len(M4):
            N4X.pop()
        if len(M4) > len(N4X):
            M4.pop()
        
        # print(len(M1))
        plt.plot(M,N1, color='red', label = 'IR1')
        plt.plot(M2,N2X, color='blue', label = 'IR2')
        plt.plot(M3,N3X, color='green', label = 'Color')
        plt.plot(M4,N4X, color='black', label = 'average')
        plt.legend()




        # print('The original start value is %f'%N1[0])
        # print('The MAF start value is %f'%yy[0])
        # print('The FIR start value is %f'%y2[0])
        # print('The N2 start value is %f'%N2[0])
        # print('The N3 start value is %f'%N3[0])
        error1 = N1[len(N1)-1] - N1[0]
        # error2 = yy[len(yy)-1] - yy[0]
        # error3 = y2[len(y2)-1] - y2[0]
        error4 = N2X[len(N2X)-1] - N2X[0]
        error5 = N3X[len(N3X)-1] - N3X[0]
        error = (error1+error4+error5)/5
        error_1 = N4X[len(N4X)-1] - N4X[0]
        print('IR1 camera error value is %.10f'%error1)
        # print('The MAF end value is %.10f'%error2)
        # print('The FIR end value is %.10f'%error3)
        print('IR2 camera error value is %.10f'%error4)
        print('Color camera error value is %.10f'%error5)
        print('The average error is %.10f'%error)
        print('The average line error is %.10f'%error_1)
        plt.xlabel('time')
        plt.ylabel('x_position')
        plt.title('time vs x_position')
        plt.show()

    
if __name__ == "__main__":
    q = Process_data()
    rospy.spin()