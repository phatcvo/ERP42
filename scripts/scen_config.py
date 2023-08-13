#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from calendar import c
from itertools import count
import sys,os
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,CtrlCmd,GetTrafficLightStatus,SetTrafficLight
from lib.utils import pathReader, findLocalPath,purePursuit,cruiseControl,vaildObject,pidController,velocityPlanning,latticePlanner
import tf
from math import cos,sin,sqrt,pow,atan2,pi
import matplotlib.pyplot as plt


class scen_config():
    def __init__(self):
        rospy.init_node('erp42_total', anonymous=True)

        arg = rospy.myargv(argv = sys.argv)
        self.traffic_control = arg[1]

        #publisher
        traffic_pub = rospy.Publisher("/SetTrafficLight", SetTrafficLight,queue_size=1)

        #def
        self.is_status=False
        self.is_obj=False
        self.is_traffic=True
        ## traffic light information (global_x, global_y, index)
        self.traffic_info = [[58.50, 1180.41 ,'C119BS010001'], 
                             [85.61, 1227.88 ,'C119BS010021'],
                             [136.58,1351.98 ,'C119BS010025'],
                             [141.02,1458.27 ,'C119BS010028'],
                             [139.39,1596.44 ,'C119BS010033'],
                             [48.71, 1208.02 ,'C119BS010005'],
                             [95.58, 1181.56 ,'C119BS010047'],
                             [104.46,1161.46 ,'C119BS010046'],
                             [85.29, 1191.77 ,'C119BS010007'],
                             [106.32,1237.04 ,'C119BS010022'],
                             [75.34, 1250.43 ,'C119BS010024'],
                             [73.62, 1218.01 ,'C119BS010012'],
                             [116.37,1190.65 ,'C119BS010040'],
                             [153.98,1371.48 ,'C119BS010073'],
                             [129.84,1385.08 ,'C119BS010039'],
                             [116.28,1367.77 ,'C119BS010074'],
                             [75.08, 1473.34 ,'C119BS010075'],
                             [67.10, 1506.66 ,'C119BS010076'],
                             [114.81,1485.81 ,'C119BS010079'],
                             [159.11,1496.63 ,'C119BS010060'],
                             [122.24,1608.26 ,'C119BS010072'],
                             [132.70,1624.78 ,'C119BS010034']]
      
        #time var
        self.count = 0
        rate = rospy.Rate(10) # 30hz

        while not rospy.is_shutdown():

            if self.is_status==True  and self.is_obj==True:

                if self.traffic_control == "True":
                    self.tl_msg.trafficLightStatus=16
                    ###################### traffic_control ######################
                    self.set_traffic_data= SetTrafficLight()
                    self.set_traffic_data.trafficLightIndex = self.tl_msg.trafficLightIndex
                    self.set_traffic_data.trafficLightStatus = 16 ##set greenlight 
                    traffic_pub.publish(self.set_traffic_data)


            # if count/300==1 :
            #     global_path_pub.publish(self.global_path)
            #     count=0
            self.count+=1

            rate.sleep()



    
if __name__ == '__main__':
    try:
        kcity_pathtracking=scen_config()
        # plt.show()
    except rospy.ROSInterruptException:
        pass
