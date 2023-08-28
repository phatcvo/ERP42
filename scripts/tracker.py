#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 2023.01.15
# Phat C. Vo in RML

import rospy
from math import pi,sqrt,pow
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
from geometry_msgs.msg import Point
import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi
import tf

global local_ref_path1
class main_planner:
    def __init__(self):
        rospy.init_node('MainAlgorithm', anonymous = True)
        # Subscriber
        # rospy.Subscriber("/global_path", Path, self.global_path_callback) 
        rospy.Subscriber("/local_selected_path", Path, self.local_selected_path_callback) 
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_callback)
        rospy.Subscriber("/local_planned_vel", CtrlCmd, self.local_planned_vel_callback)
        # Publisher
        self.ctrl_cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size = 1) 
        # local_path_pub = rospy.Publisher('/local_current_path',Path, queue_size = 1)
        self.ctrl_cmd_msg = CtrlCmd() # Publisher to ROS msg by MORAI Definition
        self.ctrl_cmd_msg.longlCmdType = 1     
        # define
        self.is_path = False # Path data check
        self.is_status = False # Ego Status data check
        self.is_local_selected_path = False
        self.status_msg = EgoVehicleStatus()
        
        # class_import
        self.tracking_control = purePursuit()
        self.vel_control = pidControl()
        
        # global lattice_path
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_status == True and self.is_local_selected_path == True:     
                self.local_path, self.current_waypoint = self.get_local_selected_path_current_waypoint(self.local_selected_path, self.status_msg)
                # print("Local path=============", self.local_ref_path)
                # local_ref_path1 = self.get_ref_local_path(self.local_ref_path)
    
                # ref_path = self.local_path
                # ref_path = self.local_ref_path
                # print("local_ref_path: ",self.local_path)
                # local_path_pub.publish(self.local_path)
                self.tracking_control.getPath(self.local_path)
                self.tracking_control.getEgoStatus(self.status_msg)
                self.ctrl_cmd_msg.steering = self.tracking_control.steering_angle()/180 * pi    
                print("Steering_pub: ",self.ctrl_cmd_msg.steering )
                output = self.vel_control.pid(self.local_planned_vel.velocity, self.status_msg.velocity.x)
                print ("Accl_pub", output)
                self.ctrl_cmd_msg.accel = output
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                
                # print("ref_path===================", self.local_path.poses.position)
            rate.sleep()
    #=============================================================== 
    ## Create current waypoint and local_ref_path using local_selected_path and vehicle status_msg ##
    def get_local_selected_path_current_waypoint(self, local_selected_path, ego_status):  
        global local_ref_path, current_waypoint, dist
        dist = 0
        local_ref_path = Path()
        current_x = ego_status.position.x
        current_y = ego_status.position.y
        current_waypoint = 0
        local_ref_path_size = 100 
        min_dist = float('inf') 

        for i in range(len(local_selected_path.poses)) :
            dx = current_x - local_selected_path.poses[i].pose.position.x
            dy = current_y - local_selected_path.poses[i].pose.position.y

            dist = sqrt(pow(dx,2) + pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                current_waypoint = i
        if current_waypoint + local_ref_path_size > len(local_selected_path.poses) :
            last_local_waypoint = len(local_selected_path.poses)
        else :
            last_local_waypoint = current_waypoint + local_ref_path_size

        local_ref_path.header.frame_id='map'
        for i in range(current_waypoint, last_local_waypoint) :
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = local_selected_path.poses[i].pose.position.x
            tmp_pose.pose.position.y = local_selected_path.poses[i].pose.position.y
            tmp_pose.pose.position.z = local_selected_path.poses[i].pose.position.z
            tmp_pose.pose.orientation.x = 0
            tmp_pose.pose.orientation.y = 0
            tmp_pose.pose.orientation.z = 0
            tmp_pose.pose.orientation.w = 1
            local_ref_path.poses.append(tmp_pose)

        return local_ref_path, current_waypoint
    
    #================================================================
    # local_selected_path Subscribe
    def local_selected_path_callback(self, msg):
        self.local_selected_path = msg
        self.is_local_selected_path = True    
        # print ("Global path")
        #=================================
    # local_path Subscribe
    def local_planned_vel_callback(self, msg):
        self.local_planned_vel = msg  
        self.is_local_planned_vel = True

        

    #================================================================
    # Ego-vehicle Status Subscribe
    def ego_status_callback(self, msg):
        self.status_msg = msg
        self.is_status = True

        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z), tf.transformations.quaternion_from_euler(0, 0, self.status_msg.heading/180*pi),rospy.Time.now(), "gps", "map")
# =====================================================================
class purePursuit:
    def __init__(self):
        self.forward_point = Point()
        self.current_position = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 1.95
        self.lfd = 5 # look ahead distance L
        self.kf = 0.95 # look forward gain
        self.min_lfd = 2
        self.max_lfd = 30
        self.steering = 0
        # self.dd = 0
        
    def getPath(self, msg):
        self.path = msg  #nav_msgs/Path 
    
    ## current coordinate of the vehicle 
    def getEgoStatus(self, msg):
        self.current_vel=msg.velocity  #kph
        self.vehicle_yaw=(msg.heading)/180*pi   # rad
        self.current_position.x = msg.position.x 
        self.current_position.y = msg.position.y 
        self.current_position.z = msg.position.z 

    # Steering calculation using purePursuit algorithm
    def steering_angle(self):
        vehicle_position = self.current_position  
        self.is_look_forward_point = False 

        # Reference of Map and vehicle coordinate conversion
        translation = [vehicle_position.x, vehicle_position.y]
        t = np.array([  [cos(self.vehicle_yaw),  -sin(self.vehicle_yaw), translation[0]], [sin(self.vehicle_yaw), cos(self.vehicle_yaw),  translation[1]], [0, 0, 1]])
        det_t = np.array([  [t[0][0], t[1][0],  -(t[0][0] * translation[0] + t[1][0] * translation[1])], [t[0][1], t[1][1], -(t[0][1] * translation[0] + t[1][1] * translation[1])], [0, 0, 1]])
        
        # Path data read  
        for i in self.path.poses :   
            path_point = i.pose.position     
            # print("pose", path_point)
            global_path_point = [path_point.x, path_point.y, 1]            
            # Convert map reference coordinate data to vehicle reference coordinate data                     
            local_path_point = det_t.dot(global_path_point)  
            if local_path_point[0] > 0:  
                dis = sqrt(pow(local_path_point[0],2) + pow(local_path_point[1],2))
                # If the distance between a specific point and vehicle is equal to or greater than Lfd, 
                # designate that location as a Look Forward Point        
                # print("current_vel: ", self.current_vel.x)       
                if dis >= self.lfd:  
                    self.lfd = self.current_vel.x * self.kf
                    # print("lfd: ", self.lfd)
                    if self.lfd < self.min_lfd :
                        self.lfd = self.min_lfd 
                    elif self.lfd > self.max_lfd : 
                        self.lfd = self.max_lfd
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    # print("Dis: {} >= {} ".format(dis, self.lfd))
                    break

        theta = atan2(local_path_point[1],local_path_point[0])                                  
        # print("theta:", theta)
        if self.is_look_forward_point:  
            # print("Look_forward_point:", self.is_look_forward_point)
            # local path tracking control based on Pure-pursuit tan^-1(2 * wheel_base * sin(theta) / Look forward distance)
            self.steering = atan2((2 * self.vehicle_length * sin(theta)), self.lfd) * 180/pi #deg
            # print("Steering_point:", self.steering)
            return self.steering
            
        else:
            # print("Look_forward_point =:", self.is_look_forward_point)
            return 0

# ====================================
class pidControl:
    def __init__(self):
        self.p_gain = 0.055
        self.i_gain = 0.00
        self.d_gain = 0.09
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.033

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel
        p_control = self.p_gain * error
        if error <= 5:
            self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime
        output = p_control + self.i_control + d_control
        self.prev_error = error
        return output
    
if __name__ == '__main__':
    # try:
    main_planner()
    # except rospy.ROSInterruptException:
        # pass
