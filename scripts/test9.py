#!/usr/bin/env python
import rospy
import time
import csv
import numpy as np
import matplotlib.pyplot as plt
from robot_control1 import GetData
import statistics
from scipy.stats import norm
import scipy.stats as st

t = [] # time
y = [] # position

with open('/home/pyu2020/IR1_300x.csv') as f:
    reader = csv.reader(f)
    for row in reader:
        t.append(float(row[0]))
        y.append(float(row[1]))

# mean = statistics.mean(y)
# sd = statistics.stdev(y)
# A = sum(y)/300
# print(A)

# # plt.plot(y, norm.pdf(y, mean, sd))
# plt.hist(y, bins=25, density=True, alpha=0.6, color='g')
# plt.plot(y, norm.pdf(y, mean, sd), 'k', linewidth=2)
# title = "Fit results: mu = %f,  std = %f" % (mean, sd)
# plt.title(title)
plt.plot(t,y)
plt.show()


