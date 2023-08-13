#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from calendar import c
from itertools import count
import sys,os
import rospy
from nav_msgs.msg import Path
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
# from lib.utils import pathReader, findLocalPath,purePursuit
import tf
from math import pi

import rospkg
from geometry_msgs.msg import PoseStamped,Point
from math import cos,sin,sqrt,pow,atan2,pi



## output path from text file ##################################
class pathReader :
    def __init__(self,pkg_name):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)



    def read_txt(self,file_name):
        full_file_name=self.file_path+"/path/"+file_name
        openFile = open(full_file_name, 'r')
        out_path=Path()
        
        out_path.header.frame_id='/map'
        line=openFile.readlines()
        for i in line :
            tmp=i.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.position.z=float(tmp[2])
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=1
            out_path.poses.append(read_pose)
        
        
        openFile.close()
        return out_path ## Returns the read path as global_path ##
      

################################################################
## Create current waypoint and local_path using global_path and vehicle status_msg ##
def findLocalPath(ref_path,status_msg): 
    out_path=Path()
    current_x=status_msg.position.x
    current_y=status_msg.position.y
    current_waypoint=0
    min_dis=float('inf')
    x_rec, y_rec = [], []

    for i in range(len(ref_path.poses)) :
        dx=current_x - ref_path.poses[i].pose.position.x
        dy=current_y - ref_path.poses[i].pose.position.y

        x_rec.append(current_x)
        y_rec.append(current_y)

        dis=sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i


    if current_waypoint+50 > len(ref_path.poses) :
        last_local_waypoint= len(ref_path.poses)
    else :
        last_local_waypoint=current_waypoint+50




    out_path.header.frame_id='map'
    for i in range(current_waypoint,last_local_waypoint) :
        tmp_pose=PoseStamped()
        tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x=0
        tmp_pose.pose.orientation.y=0
        tmp_pose.pose.orientation.z=0
        tmp_pose.pose.orientation.w=1
        out_path.poses.append(tmp_pose)

    return out_path,current_waypoint ## local_path, waypoint##

################################################################
## path tracking approach ######################################
## purePursuit controller ######################################
class purePursuit : 
    def __init__(self):
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.vehicle_length=1.8
        # self.lfd=5
        self.lfd = 5# look ahead distance L
        self.kf = 0.85 # look forward gain
        self.min_lfd = 2
        self.max_lfd = 15
        self.steering = 0
        self.dd = 0
        

    def getPath(self,msg):
        self.path=msg  #nav_msgs/Path 
    
    
    ## current coordinate of the vehicle ######################
    def getEgoStatus(self,msg):

        self.current_vel=msg.velocity  #kph
        self.vehicle_yaw=(msg.heading)/180*pi   # rad
        # print("heading : ", msg.heading)
        self.current_postion.x=msg.position.x 
        self.current_postion.y=msg.position.y 
        self.current_postion.z=msg.position.z 


    ## Steering calculation using purePursuit algorithm ##
    def steering_angle(self):
        vehicle_position=self.current_postion
        rotated_point=Point()
        self.is_look_forward_point= False
        

        for i in self.path.poses :
            path_point=i.pose.position
            dx= path_point.x - vehicle_position.x
            dy= path_point.y - vehicle_position.y
            # x_delta = path_point.x - vehicle_position.x
            # y_delta = path_point.y - vehicle_position.y
            rotated_point.x=cos(self.vehicle_yaw)*dx +sin(self.vehicle_yaw)*dy
            rotated_point.y=sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
 
            
            
            if rotated_point.x>0 :
                dis=sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
                
                if dis>= self.lfd :     
                    self.lfd=self.current_vel.x * self.kf
                    # print("lfd : ", self.lfd)
                    # print('dis: {}, lfd: {}'.format(dis, self.lfd))
                    if self.lfd < self.min_lfd : 
                        self.lfd=self.min_lfd
                    elif self.lfd > self.max_lfd :
                        self.lfd=self.max_lfd
                    self.forward_point=path_point
                    self.is_look_forward_point=True
                    break
            # print("position___ :",path_point.x)
        
        theta=atan2(rotated_point.y,rotated_point.x)

        
        if self.is_look_forward_point :
            self.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi #deg
            # plt.cla()
            # plt.plot(x_rec, y_rec, color='blue', linewidth=1)
            # plt.show()
            return self.steering
        else : 
            print("no found forward point")
            return 0


class erp_planner():
    def __init__(self):
        rospy.init_node('erp42_total', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]
        self.traffic_control=arg[2]
        

        #publisher
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1)
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1)
        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)


        ctrl_msg= CtrlCmd()
        
        #subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB)

        #def
        self.is_status=True
 
        self.is_traffic=False
  
        #class
        path_reader=pathReader('erp_py')
        #read path
        self.global_path = path_reader.read_txt(self.path_name+".txt")
        
        pure_pursuit=purePursuit() 
        
        self.status_msg = EgoVehicleStatus()


        #time var
        self.count = 0
        rate = rospy.Rate(10) # 30hz


        while not rospy.is_shutdown():

            if self.is_status==True :
                local_path, self.current_waypoint = findLocalPath(self.global_path,self.status_msg)                

                pure_pursuit.getPath(local_path)
                pure_pursuit.getEgoStatus(self.status_msg)

                ctrl_msg.steering=-pure_pursuit.steering_angle() / 180 * pi

                control_input=1.5               
                if control_input > 0 :
                    ctrl_msg.accel= control_input
                    ctrl_msg.brake= 0
                else :
                    ctrl_msg.accel= 0
                    ctrl_msg.brake= -control_input

                local_path_pub.publish(local_path)
                ctrl_pub.publish(ctrl_msg)

                self.steering_angle=ctrl_msg.steering

            
            global_path_pub.publish(self.global_path)

            self.count+=1

            rate.sleep()


    def statusCB(self,data):
        self.status_msg=EgoVehicleStatus()
        self.status_msg=data
        ## visualize ================
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, self.status_msg.heading/180*pi), rospy.Time.now(), "gps", "map")
        self.is_status=True                 

        

    
if __name__ == '__main__':
    try:
        kcity_pathtracking=erp_planner()
    except rospy.ROSInterruptException:
        pass
