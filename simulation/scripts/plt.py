#!/usr/bin/env python
# coding: UTF-8

import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Point
from functools import reduce



x = []
y = []

def callback(data):
    global x
    global y
    x.append(data.x)
    y.append(data.y)
    # print x,y

def str2float(s):
    #DIGITS = {'0':0,'1':1,'2':2,'3':3,'4':4,'5':5,'6':6,'7':7,'8':8,'9':9}
    def fn(x,y):
        return x*10 + y
    n = s.index('.')
    s1 = list(map(int,[x for x in s[:n]]))
    s2 = list(map(int,[x for x in s[n+1:]]))
    # print s1,s2
    # print float(reduce(fn,s2))/(10**len(s2))
    return reduce(fn,s1) + float(reduce(fn,s2))/(10**len(s2))    
    
def main():

    rospy.init_node('plt',anonymous=True)
    rate = rospy.Rate(10)

#    cx = np.arange(0, 100, 1)
#    cy = [math.sin(ix / 15.0) * ix / 2.0 for ix in cx]

    cx = []
    cy = []
    
    print('\'123.456\'=',str2float('123.456'))
    
    file_name = "./src/decision/data/write.txt"
    f = open(file_name,'r')
    data = f.read()
    rows = data.split('\n')
    for row in rows:
        split_row = row.split('\t')
        #print split_row,len(split_row)
        if len(split_row) > 1:
            cx.append(str2float(split_row[0]))
            cy.append(str2float(split_row[-1]))
    #print cx,cy
        
    print "Load path file: " + file_name + " successed!"

    rospy.Subscriber("/sensor/IMU/location",Point,callback)

    while not rospy.is_shutdown():
        plt.cla()
        plt.plot(cx,cy,".r",label="course")
        plt.plot(x,y,"-b",label="trajectory")
        # print "here is while"
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)
        # rospy.spin()
        rate.sleep()


if __name__ == "__main__":
    main()
