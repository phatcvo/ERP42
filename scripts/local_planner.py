#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @date 2023-01
# @author Phat C. Vo in RML

import rospy
import os
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, GetTrafficLightStatus, SetTrafficLight, CtrlCmd
from std_msgs.msg import Int32
import numpy as np
import tf

import csv
import pandas as pd
cost1_rec = []
cost2_rec = []
cost3_rec = []
cost4_rec = []
cost5_rec = []
cost6_rec = []
cost7_rec = []
cost8_rec = []
cost9_rec = []
cost10_rec = []
cost11_rec = []
class local_planner:
    def __init__(self):
        rospy.init_node('local_planner', anonymous = True)
        # Subscriber
        rospy.Subscriber("/global_path", Path, self.global_path_callback) 
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.getTL_callback)

        # Publisher
        self.local_selected_path = rospy.Publisher('/local_selected_path',Path, queue_size=1)
        self.local_planned_vel = rospy.Publisher('/local_planned_vel', CtrlCmd, queue_size = 1)
        self.planned_vel_msg = CtrlCmd() # Publisher to ROS msg by MORAI Definition
        # define
        self.is_path = False # Path data check
        self.is_status = False # Ego Status data check
        self.is_global_path = False
        self.is_obj = False
        self.is_lattice_planning = False
        self.ACC_type = ["Pedestrian", "NPC-vehicle", "Obstacle", "Traffic light", "OFF"]
        self.Obj_type = ["Pedestrian", "NPC-vehicle", "Obstacle", "Traffic light", "OFF"]
        # arg = rospy.myargv(argv=sys.argv)
        # self.path_name=arg[1]
        
        CAR_MAX_SPEED = 20 #km/h
        self.is_traffic = False
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


        self.object_info_msg = ObjectStatusList()
        self.status_msg = EgoVehicleStatus()
        
        # class_import
        self.vel_planning = velocityPlanning(CAR_MAX_SPEED, 1)
        # self.vel_control = pidControl()
        self.cruise_ctrl = cruiseControl(0.5,1.0)
        
        self.lattice = latticePlanner()
        self.forward_object_detector = ForwardObjectDetector(self.traffic_info)
        
        global lattice_path
        
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_status == True and self.is_global_path == True: 
                
                # == Path data read ===================================  
                self.local_path, self.current_waypoint = self.get_local_path_current_waypoint(self.global_path, self.status_msg)

                # == object detection ========================================
                global_npc_info, local_npc_info, global_ped_info, local_ped_info, global_obs_info, local_obs_info = self.forward_object_detector.detect_object(self.status_msg,self.object_data) 
                
                # == velocity planning ============================
                # target_vel = self.vel[self.current_waypoint]  
                self.planned_vel_msg.velocity = self.vel[self.current_waypoint]  
                # self.cruise_ctrl.checkObject(self.local_path, global_npc_info, local_npc_info ,global_ped_info, local_ped_info, global_obs_info, local_obs_info)  
                # cc_vel = self.cruise_ctrl.acc(local_npc_info, local_ped_info, local_obs_info, self.status_msg.velocity.x, CAR_MAX_SPEED)

                # == local planner ====================================================================
                lattice_path = self.lattice.latticePlanner(self.local_path, self.status_msg)
                selected_lane = self.lattice.select_lane(global_npc_info, global_obs_info, lattice_path)
                self.is_lattice_planning = self.lattice.checkObject(self.local_path, global_npc_info, global_obs_info, lattice_path)
                
                # Speed control using PID (target Velocity, Status Velocity)  
                # if self.is_lattice_planning:
                if selected_lane != -1:
                    ref_path = lattice_path[selected_lane]
                # else:
                #     ref_path = self.local_path
                
                for i in range(len(lattice_path)): 
                    globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1), Path, queue_size = 1)
                    globals()['lattice_pub_{}'.format(i+1)].publish(lattice_path[i])

            
                # self.tracking_control.getPath(self.local_path)
                # self.tracking_control.getPath(ref_path)
                # self.tracking_control.getEgoStatus(self.status_msg)
                # self.ctrl_cmd_msg.steering = self.tracking_control.steering_angle()/180 * pi    

                # ==========================
                # Speed control using PID (target Velocity, Status Velocity)
                # ref_vel = min(target_vel, cc_vel)
                # ref_vel = target_vel.data
                # self.ctrl_cmd_msg.accel = 1.0
                # self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                # print(self.planned_vel_msg.velocity)
                self.local_planned_vel.publish(self.planned_vel_msg)
                # output = self.vel_control.pid(min(target_vel, cc_vel), self.status_msg.velocity.x)
                # output = self.vel_control.pid(target_vel, self.status_msg.velocity.x)

                # if output > 0.0:
                #     self.ctrl_cmd_msg.accel = output
                #     # self.ctrl_cmd_msg.velocity = 50.0
                #     self.ctrl_cmd_msg.brake = 0.0
                # else:
                #     # self.ctrl_cmd_msg.accel = 0.0
                #     self.ctrl_cmd_msg.velocity = 5.0
                #     self.ctrl_cmd_msg.brake = 0

                # self.local_sub_path.publish(lattice_path[lattice_path_num])
                self.local_selected_path.publish(ref_path)
                # print("Local path", ref_path)
                # self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                # self.print_info()
            rate.sleep()
    #=================================   
    ## Create current waypoint and local_path using global_path and vehicle status_msg ##
    def get_local_path_current_waypoint(self, global_path, ego_status):  
        global local_path, current_waypoint, dist
        dist = 0
        local_path = Path()
        current_x = ego_status.position.x
        current_y = ego_status.position.y
        current_waypoint = 0
        local_path_size = 100 
        min_dist = float('inf') 

        for i in range(len(global_path.poses)) :
            dx = current_x - global_path.poses[i].pose.position.x
            dy = current_y - global_path.poses[i].pose.position.y

            dist = sqrt(pow(dx,2) + pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                current_waypoint = i
            # print(dist)
        if current_waypoint + local_path_size > len(global_path.poses) :
            last_local_waypoint = len(global_path.poses)
        else :
            last_local_waypoint = current_waypoint + local_path_size

        local_path.header.frame_id='map'
        for i in range(current_waypoint, last_local_waypoint) :
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = global_path.poses[i].pose.position.x
            tmp_pose.pose.position.y = global_path.poses[i].pose.position.y
            tmp_pose.pose.position.z = global_path.poses[i].pose.position.z
            tmp_pose.pose.orientation.x = 0
            tmp_pose.pose.orientation.y = 0
            tmp_pose.pose.orientation.z = 0
            tmp_pose.pose.orientation.w = 1
            local_path.poses.append(tmp_pose)

        return local_path, current_waypoint ## local_path, waypoint##
    #=================================
    # object Subscribe
    def object_callback(self, msg):
        self.is_obj = True
        self.object_data = msg

        self.object_num = msg.num_of_npcs + msg.num_of_obstacle + msg.num_of_pedestrian
    #=================================
    # # local_path Subscribe
    # def local_path_callback(self, msg):
    #     self.is_path = True
    #     self.path = msg  
    #     # print("global_path_callback===") 
    #     if len(self.path.poses) < 7:
    #         self.is_path = False
    #=================================
    # global_path Subscribe
    def global_path_callback(self, msg):
        self.global_path = msg
        self.vel = self.vel_planning.curvedBaseVelocity(self.global_path, 150)
        self.is_global_path = True    
        # print("global_path_callback ======", self.is_global_path)  
    #=================================
    # Ego-vehicle Status Subscribe
    def ego_status_callback(self, msg):
        self.status_msg = EgoVehicleStatus()
        self.is_status = True
        self.status_msg = msg
        # print("self.is_status  ======", self.is_status) 
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, self.status_msg.heading/180*pi),
                        rospy.Time.now(), "gps", "map")
    #=================================
    # Traffic light Subscribe
    def getTL_callback(self, msg):
        self.is_traffic = True
        self.tl_msg = GetTrafficLightStatus()
        self.tl_msg = msg
    #=================================
    def print_info(self):
        
        os.system('clear')
        print('--------------------status-------------------------')
        print('position: {:.1f}, {:.1f}, {:.1f}'.format(self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z))
        print('velocity: {:.1f}, {:.1f}, {:.1f} km/h'.format(self.status_msg.velocity.x * 3.6, self.status_msg.velocity.y * 3.6, self.status_msg.velocity.z * 3.6))
        print('heading :{:.3f} deg'.format(self.status_msg.heading))
        
        print('--------------------controller-------------------------')
        print("Target Velocity: {:.1f} km/h, {}". format(out_vel_plan[0], out_vel))
        print("Acceleration {:.1f} m/s^2, Brake {:.1f}, steering {:.1f} deg".format(self.ctrl_cmd_msg.accel, self.ctrl_cmd_msg.brake, self.ctrl_cmd_msg.steering))
        print("ACC Mode: ", self.ACC_type[acc_stu])
        print("Look_distance: {} m".format(look_distance))
        print("Num_selected_lane: {}".format(selected_lane))

        

        print('--------------------localization-------------------------')
        print('all waypoint size: {:.3f} '.format(len(self.global_path.poses)))
        print('current waypoint : {:.3f} '.format(self.current_waypoint))
        
        print('--------------------Object-------------------------')
        print("Check Object: ", self.Obj_type[obj_stu])
        print("Dis to select lane: ", dis_select_obj)
        print("Distance nearest object: {:.3f} m".format(dis_check_obj))
        print("Lane weight:== ", lane_weight)
        print('Lane No. : {}'.format(selected_lane))
        print('No. of Lane: {}'.format(len(out_path)))
        
        print("NPC-vehicle: {}".format(num_of_npcs))
        print("Person: {}".format(num_of_pedestrian))
        print("Obstacle: {}".format(num_of_obstacle))
        if self.is_traffic == True:
            print('--------------------trafficLight-------------------------')
            print('traffic index : {}'.format(self.tl_msg.trafficLightIndex))
            print('traffic type : {}'.format(self.tl_msg.trafficLightType))
            print('traffic status : {}'.format(self.tl_msg.trafficLightStatus))
        

        # cost1_rec.append(dis_check_obj)
        # cost2_rec.append(dis_select_obj)
        # cost3_rec.append(selected_lane)
        # cost = [self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z]
        # cost4_rec.append(cost)
        # cost5_rec.append(lane_weight[0])
        # cost6_rec.append(lane_weight[1])
        # cost7_rec.append(lane_weight[2])
        # cost8_rec.append(lane_weight[3])
        # cost9_rec.append(lane_weight[4])
        # cost10_rec.append(lane_weight[5])
        # cost11_rec.append(lane_weight[6])

        # raw_data = {'dis_check': cost1_rec,
        #         'dis_select': cost2_rec,
        #         'selected_lane': cost3_rec,
        #         'position': cost4_rec,
        #         'lane_weight1': cost5_rec,
        #         'lane_weight2': cost6_rec,
        #         'lane_weight3': cost7_rec,
        #         'lane_weight4': cost8_rec,
        #         'lane_weight5': cost9_rec,
        #         'lane_weight6': cost10_rec,
        #         'lane_weight7': cost11_rec,
        #         }

        # df = pd.DataFrame(raw_data, columns = ['dis_check', 'dis_select', 'position', 'selected_lane',
        #                                   'lane_weight1',  'lane_weight2','lane_weight3','lane_weight4','lane_weight5',
        #                                   'lane_weight6',
        #                                   'lane_weight7',])    


        # df = pd.DataFrame(raw_data)  
        # # saving the dataframe    
        # df.to_csv(r'/home/rml-phat/catkin_ws/raw_data.csv', index=False)
        # print(df)

# ====================================  
class velocityPlanning:
    def __init__ (self, car_max_speed, road_friction):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction

    def curvedBaseVelocity(self, global_path, point_num):
        global out_vel_plan
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(global_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = global_path.poses[i+box].pose.position.x
                y = global_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T


            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a+b*b-c)
            v_max = sqrt(r*9.8*self.road_friction)
            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses) - point_num, len(global_path.poses) - 10):
            out_vel_plan.append(50)

        for i in range(len(global_path.poses) - 10, len(global_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan #m/s
# # ====================================
# class pidControl:
#     def __init__(self):
#         self.p_gain = 0.055
#         self.i_gain = 0.00
#         self.d_gain = 0.09
#         self.prev_error = 0
#         self.i_control = 0
#         self.controlTime = 0.033

#     def pid(self,target_vel, current_vel):
#         error = target_vel - current_vel
#         p_control = self.p_gain * error
#         if error <= 5:
#             self.i_control += self.i_gain * error * self.controlTime
#         d_control = self.d_gain * (error-self.prev_error) / self.controlTime
#         output = p_control + self.i_control + d_control
#         self.prev_error = error
#         return output
# ====================================
## Check for obstacles (vehicle, person, stop line signal) #####
class ForwardObjectDetector:
    def __init__(self, stop_line = []):
        self.stop_line = stop_line
        

    def detect_object(self, ego_pose, object_data):
        global num_of_npcs, num_of_pedestrian, num_of_obstacle
        num_of_npcs = 0
        num_of_pedestrian = 0
        num_of_obstacle = 0
        self.all_object = object_data        
        ego_pose_x = ego_pose.position.x
        ego_pose_y = ego_pose.position.y
        ego_heading = ego_pose.heading/180*pi
        
        global_npc_info = []
        local_npc_info  = []
        global_ped_info = []
        local_ped_info  = []
        global_obs_info = []
        local_obs_info  = []   

        
        # num_of_npcs = self.all_object.num_of_npcs
        # num_of_pedestrian = self.all_object.num_of_pedestrian
        # num_of_obstacle = self.all_object.num_of_obstacle 
        
        
        

        num_of_object = self.all_object.num_of_npcs + self.all_object.num_of_pedestrian + self.all_object.num_of_obstacle         
        if num_of_object > 0:

            #translation
            tmp_theta = ego_heading
            tmp_translation = [ego_pose_x, ego_pose_y]
            tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                            [sin(tmp_theta),  cos(tmp_theta), tmp_translation[1]],
                            [0             ,               0,                  1]])
            tmp_det_t = np.array([[tmp_t[0][0], tmp_t[1][0], -(tmp_t[0][0] * tmp_translation[0] + tmp_t[1][0]*tmp_translation[1])],
                                [tmp_t[0][1], tmp_t[1][1], -(tmp_t[0][1] * tmp_translation[0] + tmp_t[1][1]*tmp_translation[1])],
                                [0,     0,      1]])           

            # npc vehicle translation        
            for self.npc_list in self.all_object.npc_list:
                global_result = np.array([[self.npc_list.position.x],[self.npc_list.position.y],[1]])
                local_result = tmp_det_t.dot(global_result)
                if local_result[0][0] > 0:        
                    global_npc_info.append([self.npc_list.type, self.npc_list.position.x, self.npc_list.position.y, self.npc_list.velocity.x])
                    local_npc_info.append([self.npc_list.type, local_result[0][0], local_result[1][0], self.npc_list.velocity.x])
                    
            num_of_npcs = len(local_npc_info)
                # self.npc = [self.npc_list.type, self.all_object.num_of_npcs]
                # print("========== {}".format(self.npc))
                # print(self.ACC_type[self.npc_list.type] + ": {}".format(self.all_object.num_of_npcs))

            # ped translation
            for self.ped_list in self.all_object.pedestrian_list:
                global_result = np.array([[self.ped_list.position.x],[self.ped_list.position.y],[1]])
                local_result = tmp_det_t.dot(global_result)
                if local_result[0][0] > 0:
                    global_ped_info.append([self.ped_list.type, self.ped_list.position.x, self.ped_list.position.y, self.ped_list.velocity.x])
                    local_ped_info.append([self.ped_list.type, local_result[0][0], local_result[1][0], self.ped_list.velocity.x])
                    
            num_of_pedestrian = len(local_ped_info)
                # self.ped = [self.ped_list.type, self.all_object.num_of_pedestrian]
                # print("========== {}".format(self.ped))
                # print(self.ACC_type[self.ped_list.type] + ": {}".format(self.all_object.num_of_pedestrian))

            # obs translation
            for self.obs_list in self.all_object.obstacle_list:
                global_result = np.array([[self.obs_list.position.x], [self.obs_list.position.y], [1]])
                local_result = tmp_det_t.dot(global_result)
                if local_result[0][0] > 0:
                    global_obs_info.append([self.obs_list.type, self.obs_list.position.x, self.obs_list.position.y, self.obs_list.velocity.x])
                    local_obs_info.append([self.obs_list.type, local_result[0][0], local_result[1][0], self.obs_list.velocity.x])
            num_of_obstacle = len(local_obs_info)
                # print(self.ACC_type[self.obs_list.type] + ": {}".format(self.all_object.num_of_obstacle))

        #     global_obj_info = [global_npc_info, global_ped_info, global_obs_info]
        #     local_obj_info = [local_npc_info, local_ped_info, local_obs_info]

        # return global_obj_info, local_obj_info
        
        return global_npc_info, local_npc_info, global_ped_info, local_ped_info, global_obs_info, local_obs_info
# ====================================
class cruiseControl: ## ACC(advanced cruise control) ##
    def __init__(self, object_vel_gain, object_dis_gain):
        self.npc_vehicle = [False, 0]
        self.object = [False, 0]
        self.Person = [False, 0]
        self.object_vel_gain = object_vel_gain
        self.object_dis_gain = object_dis_gain

    ## Check for obstacles on the path (vehicle, person, obstacle) ##
    def checkObject(self, ref_path, global_npc_info, local_npc_info, global_ped_info, local_ped_info, global_obs_info, local_obs_info):

        min_rel_distance = float('inf')

        if len(global_ped_info) > 0 :        
            for i in range(len(global_ped_info)):
                for path in ref_path.poses :      
                    if global_ped_info[i][0] == 0 : # type=0 [pedestrian]                    
                        dis = sqrt(pow(path.pose.position.x - global_ped_info[i][1],2) + pow(path.pose.position.y - global_ped_info[i][2], 2))
                        if dis < 2.35:                            
                            rel_distance = sqrt(pow(local_ped_info[i][1],2)+pow(local_ped_info[i][2],2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.Person = [True, i]
        
        if len(global_npc_info) > 0 :            
            for i in range(len(global_npc_info)):
                for path in ref_path.poses :      
                    if global_npc_info[i][0] == 1 : # type=1 [npc_vehicle] 
                        dis = sqrt(pow(path.pose.position.x-global_npc_info[i][1],2) + pow(path.pose.position.y-global_npc_info[i][2],2))
                        if dis < 2.35:
                            rel_distance= sqrt(pow(local_npc_info[i][1],2)+pow(local_npc_info[i][2],2))                            
                            if rel_distance < min_rel_distance:
                                min_rel_distance=rel_distance
                                self.npc_vehicle = [True, i]

        if len(global_obs_info) > 0 :            
            for i in range(len(global_obs_info)):
                for path in ref_path.poses :      
                    if global_obs_info[i][0] == 2 : # type=1 [obstacle] 
                        dis = sqrt(pow(path.pose.position.x - global_obs_info[i][1],2) + pow(path.pose.position.y-global_obs_info[i][2],2))
                        if dis < 2.35:
                            rel_distance = sqrt(pow(local_obs_info[i][1],2)+pow(local_obs_info[i][2],2))                        
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.object = [True, i]  

        # for path in ref_path.poses:
        #     if global_valid_object[i][0]==3 :
        #                 traffic_sign='STOP'

        #                 if len(tl)!=0  and  global_valid_object[i][3] == tl[0] :
        #                     if tl[1] == 48 or tl[1]==16   :   #
        #                         traffic_sign ='GO'
        #                 # print(traffic_sign)
        #                 if traffic_sign =='STOP':
        #                     dis=sqrt(pow(path.pose.position.x-global_valid_object[i][1],2)+pow(path.pose.position.y-global_valid_object[i][2],2))
                            
        #                     if dis<2.5 :
        #                         rel_distance= sqrt(pow(local_valid_object[i][1],2)+pow(local_valid_object[i][2],2))
        #                         if rel_distance < min_rel_distance:
        #                             min_rel_distance=rel_distance
        #                             self.traffic=[True, i]


    # advanced cruise control for velocity planning
    def acc(self,local_npc_info, local_ped_info, local_obs_info, ego_vel, target_vel): 

        global acc_stu, out_vel
        out_vel = target_vel
        acc_based_vel = out_vel
        default_space = 8
        time_gap = 0.3
        v_gain = self.object_vel_gain
        x_errgain = self.object_dis_gain 
        dis_safe = ego_vel * time_gap + default_space
        dis_rel = 0
        
        # print(local_ped_info[self.Person[1]][3])
        if self.Person[0] == True and len(local_ped_info) > 0: #ACC ON_Pedestrian
            # print("ACC ON Pedestrian")
            acc_stu = 0
            Pedestrian = [local_ped_info[self.Person[1]][1], local_ped_info[self.Person[1]][2], local_ped_info[self.Person[1]][3]]
            
            
            dis_rel = sqrt(pow(Pedestrian[0],2) + pow(Pedestrian[1],2))            
            vel_rel = (Pedestrian[2] - ego_vel)              
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

            acc_based_vel = ego_vel + acceleration

        # else: 
        #     print("ACC OFF Pedestrian")
        #     out_vel = pre_out_vel
        
        elif self.npc_vehicle[0] == True and len(local_npc_info) > 0: #ACC ON_vehicle   
            # print("ACC ON NPC_Vehicle")    
            acc_stu = 1     
            front_vehicle = [local_npc_info[self.npc_vehicle[1]][1], local_npc_info[self.npc_vehicle[1]][2], local_npc_info[self.npc_vehicle[1]][3]]
            
            dis_rel = sqrt(pow(front_vehicle[0],2) + pow(front_vehicle[1],2))            
            vel_rel=((front_vehicle[2] / 3.6) - ego_vel)                        
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)

            acc_based_vel = ego_vel + acceleration     
        # else: 
        #     print("ACC OFF NPC_Vehicle") 
        #     out_vel = pre_out_vel

        elif self.object[0] == True and len(local_obs_info) > 0: #ACC ON_obstacle     
            # print("ACC ON Obstacle")    
            acc_stu = 2           
            Obstacle = [local_obs_info[self.object[1]][1], local_obs_info[self.object[1]][2], local_obs_info[self.object[1]][3]]

            dis_rel = sqrt(pow(Obstacle[0], 2) + pow(Obstacle[1], 2))            
            vel_rel = (Obstacle[2] - ego_vel)
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)  
            # acceleration = vel_rel * 0.1 - 1.2* (dis_safe - dis_rel)      

            acc_based_vel = ego_vel + acceleration 
        # else: 
        #     print("ACC OFF Obstacle") 
        #     out_vel = pre_out_vel
        # if self.Person[0] == False and self.npc_vehicle[0] == False and self.object[0] == False:
        else:
            # print("ACC OFF") 
            acc_stu = 4
            acc_based_vel = target_vel
        
        if acc_based_vel > target_vel : 
            acc_based_vel = target_vel
            
        if dis_safe > dis_rel:
            out_vel = acc_based_vel
        elif acc_based_vel < target_vel :
            out_vel = acc_based_vel
        if dis_rel < default_space :
                out_vel = 0

        return out_vel
# ====================================
class latticePlanner:
    def latticePlanner(self,ref_path, vehicle_status):
        global look_distance, out_path
        out_path = []
        vehicle_pose_x = vehicle_status.position.x
        vehicle_pose_y = vehicle_status.position.y
        vehicle_velocity = vehicle_status.velocity.x*3.6

        # look_distance = int(vehicle_velocity*0.2*2)
        look_distance = int(vehicle_velocity*0.2*3)
        # print("look", look_distance)
        if look_distance < 10 : #min 10m   
            look_distance = 10            
        elif look_distance > 30:
            look_distance = 30

        if len(ref_path.poses) > look_distance:
            
            global_ref_start_point = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)
            global_ref_end_point = (ref_path.poses[look_distance].pose.position.x, ref_path.poses[look_distance].pose.position.y)
            # global_ref_end_point = (ref_path.poses[look_distance*2].pose.position.x, ref_path.poses[look_distance*2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            t = np.array([[cos(theta), -sin(theta), translation[0]], [sin(theta), cos(theta), translation[1]], [0, 0, 1]])
            det_t = np.array([[t[0][0], t[1][0], -(t[0][0] * translation[0] + t[1][0] * translation[1])], [t[0][1], t[1][1], -(t[0][1] * translation[0] + t[1][1] * translation[1])],[0, 0, 1]])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_t.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_t.dot(world_ego_vehicle_position)
            # lane_off_set = [-5.0, -3.0, -1.75, -1.0, 0.0, 1.0, 1.75, 3.0, 5.0]
            lane_off_set = [2.4,1.8,1.2,0.6,0,-0.6,-1.2,-1.8,-2.4]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
                
            for end_point in local_lattice_points :
                lattice_path = Path()
                lattice_path.header.frame_id = 'map'
                x = []
                y = []
                x_interval = 0.5
                xs = 0
                xf = end_point[0]
                ps = local_ego_vehicle_position[1][0]

                pf = end_point[1]
                x_num = xf / x_interval

                for i in range(xs, int(x_num)) : 
                    x.append(i * x_interval)
                
                a = [0.0, 0.0, 0.0, 0.0]
                a[0] = ps
                a[1] = 0
                a[2] = 3.0 * (pf - ps) / (xf * xf)
                a[3] = -2.0 * (pf - ps) / (xf * xf * xf)

                for i in x:
                    result = a[3] * i * i * i + a[2] * i * i + a[1] * i + a[0]
                    y.append(result)
                # print ("result", result)

                for i in range(0,len(y)) :
                    local_result = np.array([[x[i]], [y[i]], [1]])
                    global_result = t.dot(local_result)

                    read_pose = PoseStamped()
                    read_pose.pose.position.x = global_result[0][0]
                    read_pose.pose.position.y = global_result[1][0]
                    read_pose.pose.position.z = 0
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1
                    lattice_path.poses.append(read_pose)

                out_path.append(lattice_path)

            # Add_point            
            # add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses) ) 
            add_point_size = int(vehicle_velocity * 2) * 3
            # print (len(ref_path.poses))        
            
            for i in range(look_distance, add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y,ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]], [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]], [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = global_result[0][0]
                        read_pose.pose.position.y = global_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        out_path[lane_num].poses.append(read_pose)
            
            # for i in range(len(out_path)):          
            #     globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1),Path,queue_size=1)
            #     globals()['lattice_pub_{}'.format(i+1)].publish(out_path[i])
        
        return out_path

    def checkObject(self, ref_path, global_npc, global_obs, lattice_path):    
        global obj_stu, dis_check_obj
        obj_stu = 4
        dis_check_obj = 0
        
        global_obj = global_obs + global_npc
        
        check_lens = len(lattice_path[0].poses)
        lattice = False
        for obj in global_obj:
            if obj[0] == 1:
                obj_stu = 1
            elif obj [0] == 2:
                obj_stu = 2
        if len(global_obj) > 0:
            # for i in range(len(global_obj)):
            for index, path in enumerate(ref_path.poses) :   
                if index >= check_lens:
                    break
                else:
                    dis_check_obj = sqrt(pow(path.pose.position.x - global_obj[0][1], 2) + pow(path.pose.position.y - global_obj[0][2], 2))
                    if dis_check_obj < 5.35:
                        lattice = True
                        break
        return lattice

    def select_lane(self, global_npc, global_obs, out_path):
        global selected_lane, dis_select_obj, lane_weight
        selected_lane = -1     
        dis_select_obj = 0  
        lane_weight = [7, 5, 3, 1, 0, 1, 3, 5, 7] #reference path 
        global_obj = global_obs + global_npc
        for obj in global_obj:                         
            for path_num in range(len(out_path)) :                          
                for path_pos in out_path[path_num].poses :                                
                    dis_select_obj = sqrt(pow(obj[1] - path_pos.pose.position.x, 2) + pow(obj[2]-path_pos.pose.position.y, 2))

                    if dis_select_obj < 2.5:
                        lane_weight[path_num] = lane_weight[path_num] + 1
                        
        selected_lane = lane_weight.index(min(lane_weight))    
        return selected_lane
# ====================================

if __name__ == '__main__':
    local_planner()

