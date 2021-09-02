#!/usr/bin/env python
import rospy
import time
import csv
import numpy as np
import matplotlib.pyplot as plt
import math
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
            # IR1_X = self.get_apriltag_Value('IR1_x')
            # IR1_Y = self.get_apriltag_Value('IR1_y')
            # IR1_Z = self.get_apriltag_Value('IR1_z')

            # IR2_X = self.get_apriltag_Value('IR2_x')
            # IR2_Y = self.get_apriltag_Value('IR2_y')
            # IR2_Z = self.get_apriltag_Value('IR2_z')

            # Color_X = self.get_apriltag_Value('Color_x')
            # Color_Y = self.get_apriltag_Value('Color_y')
            # Color_Z = self.get_apriltag_Value('Color_z')

            Fish1_X = self.get_apriltag_Value('Fish1_x')
            Fish1_Y = self.get_apriltag_Value('Fish1_y')
            Fish1_Z = self.get_apriltag_Value('Fish1_z')

            Fish2_X = self.get_apriltag_Value('Fish2_x')
            Fish2_Y = self.get_apriltag_Value('Fish2_y')
            Fish2_Z = self.get_apriltag_Value('Fish2_z')

            T265_X = self.get_apriltag_Value('T265_x')
            T265_Y = self.get_apriltag_Value('T265_y')
            T265_Z = self.get_apriltag_Value('T265_z')

            # print('The apriltag x value in IR1 camera is %f'%IR1_X)
            # # print('The apriltag y value in IR1 camera is %f'%IR1_Y)
            # # print('The apriltag z value in IR1 camera is %f'%IR1_Z)

            # # print('The apriltag x value in IR2 camera is %f'%IR2_X)
            # # print('The apriltag y value in IR2 camera is %f'%IR2_Y)
            # # print('The apriltag z value in IR2 camera is %f'%IR2_Z)

            # # print('The apriltag x value in Color camera is %f'%Color_X)
            # # print('The apriltag y value in Color camera is %f'%Color_Y)
            # # print('The apriltag z value in Color camera is %f'%Color_Z)


            # D1 = math.sqrt((Fish1_X - T265_X)**2 + (Fish1_Y - T265_Y)**2 + (Fish1_Z - T265_Z)**2)
            # D2 = math.sqrt((Fish2_X - T265_X)**2 + (Fish2_Y - T265_Y)**2 + (Fish2_Z - T265_Z)**2)
            # D3 = (D1+D2)/2
            print("The position of apriltag in Fish1 camera is: ")
            print("              x: %f"%Fish1_X)
            print("              y: %f"%Fish1_Y)
            print("              z: %f"%Fish1_Z)
            print("The position of apriltag in Fish2 camera is: ")
            print("              x: %f"%Fish2_X)
            print("              y: %f"%Fish2_Y)
            print("              z: %f"%Fish2_Z)
            print("The position of T265 is: ")
            print("              x: %f"%T265_X)
            print("              y: %f"%T265_Y)
            print("              z: %f"%T265_Z)

            if Fish1_X > -1000 and Fish1_Y > -1000:
                D1 = math.sqrt((Fish1_X - T265_X)**2 + (Fish1_Y - T265_Y)**2 + (Fish1_Z - T265_Z)**2)
                print("The distance between apriltag and Fish1 camera is: %f"%D1)
            else:
                print("The Fish1 camera cannot detect the apriltag")

            if Fish2_X > -1000 and Fish2_Y > -1000:
                D2 = math.sqrt((Fish2_X - T265_X)**2 + (Fish2_Y - T265_Y)**2 + (Fish2_Z - T265_Z)**2)
                print("The distance between apriltag and Fish2 camera is: %f"%D2)
            else:
                print("The Fish2 camera cannot detect the apriltag")

            # print("The distance between apriltag and Fish1 camera is: %f"%D1)
            # print("The distance between apriltag and Fish2 camera is: %f"%D2)
            # print("The distance between apriltag and T265 is: ")
            # print("                Distance: %f"%D3)

            
            # i = i+1
            # A = self.get_apriltag_Value('IR1_x')*1000
            # B = self.get_apriltag_Value('IR2_x')*1000
            # C = self.get_apriltag_Value('Color_x')*1000
            # if i < 301:
            #     N1.append(A)
            #     N2.append(B)
            #     N3.append(C)
            # else:
            #     break

            print("--------------------------------------------------------------")

            time.sleep(0.1) # change the sleep time to whatever is the appropriate control rate for simulation
        
        
        # end = time.time()
        # T = end-start
        # print('The time is', T)
        # k = T/300
        # j = float(0)
        # while j < T:
        #     j = j+k
        #     M.append(j)
    
        # # print(N2[0]-N1[0])
        # # print(N2[0])
        # # print(N3[0]-N1[0])

        # N2X =[]
        # i3 = 0

        # for i3 in range(299):
        #     i3 +=1
        #     N2_1 = N2[i3]-N2[0]+N1[0]
        #     N2X.append(N2_1)

        # M1= M
        # M1.pop()


        # N3X =[]
        # i4 = 0

        # for i4 in range(299):
        #     i4 +=1
        #     N3_1 = N3[i4]-N3[0]+N1[0]
        #     N3X.append(N3_1)

        # M2= M
        # M2.pop()



        # # write csv file

        # # filename = "IR1_300x.csv"

        # # rows = np.transpose([M, N1])

        # # with open(filename, 'w') as csvfile:
        # #     csvwriter = csv.writer(csvfile)
        # #     csvwriter.writerows(rows)

        # # N1.sort()



        # # Moving Average filter (MAF)
        # X = 8
        # ii = 1
        # MM = []
        # AA = N1
        # while ii < len(AA)-X+1:
        #     BB = AA[-(ii+X):-ii]
        #     BB_ave = sum(BB)/X
        #     MM.append(BB_ave)
        #     ii += 1

        # NN = MM[::-1]
        # Y1 = []
        # for ii1 in range (X):
        #     Y1.append(N1[0])
        #     ii1 +=1
        # yy = np.hstack((Y1, NN))
        # # plt.plot(M, yy, color='red', label = 'MAF')

        # # Finite Impulse Response Filter ( FIR )
        # A2 = 0.85
        # B2 = round(1-A2, 3)

        # i2 = 0
        # new_ave = N1[0]
        # M2 = []
        # while i2 < len(N1):
        #     new_ave = A2*new_ave + B2*N1[i2]
        #     M2.append(new_ave)
        #     i2 +=1
        # y2 = M2

        # # plt.plot(M, y2, color='green', label = 'FIR')
        # if len(N1) > len(M):
        #     N1.pop()
        # if len(N1) > len(M):
        #     N1.pop()
        

        # plt.plot(M,N1, color='yellow', label = 'origin')




        # # print('The original start value is %f'%N1[0])
        # # print('The MAF start value is %f'%yy[0])
        # # print('The FIR start value is %f'%y2[0])
        # # print('The N2 start value is %f'%N2[0])
        # # print('The N3 start value is %f'%N3[0])
        # error1 = N1[len(N1)-1] - N1[0]
        # error2 = yy[len(yy)-1] - yy[0]
        # error3 = y2[len(y2)-1] - y2[0]
        # error4 = N2[len(N2)-1] - N2[0]
        # error5 = N3[len(N3)-1] - N3[0]
        # error = (error1+error2+error3+error4+error5)/5
        # print('The original end value is %.10f'%error1)
        # print('The MAF end value is %.10f'%error2)
        # print('The FIR end value is %.10f'%error3)
        # print('The N2 end value is %.10f'%error4)
        # print('The N3 end value is %.10f'%error5)
        # print('The average error is %.10f'%error)
        # plt.plot(M1,N2X, color='blue')
        # plt.plot(M2,N3X, color='black')
        # plt.xlabel('time')
        # plt.ylabel('x_position')
        # plt.title('time vs x_position')
        # plt.show()

    
if __name__ == "__main__":
    q = Process_data()
    rospy.spin()