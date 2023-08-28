#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 2023.01.15
# Phat C. Vo in RML

import rospy

from math import cos,sin,pi,sqrt,pow,atan2

from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, CtrlCmd, GetTrafficLightStatus, SetTrafficLight

import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class perception:
    def __init__(self):
        rospy.init_node('perception', anonymous = True)
        # Subscriber
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.getTL_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_callback)

        # Publisher
        self.global_info_pub = rospy.Publisher("/global_info_pub", ObjectStatusList, queue_size = 1000) # Vehicle Control Ctrl Cmd Publisher
        # self.traffic_pub = rospy.Publisher("/SetTrafficLight", SetTrafficLight, queue_size = 1)


        # define
        self.is_status = False # Ego Status data check
        self.is_global_path = False
        self.is_obj = False


        self.Obj_type = ["Pedestrian", "NPC-vehicle", "Obstacle", "Traffic light", "OFF"]
        
        # self.object_info_msg = ObjectStatusList()
        self.status_msg = EgoVehicleStatus()
        
        # class_import
        self.forward_object_detector = ForwardObjectDetector()
        rate = rospy.Rate(30) # 30hz

        while not rospy.is_shutdown():
            if self.is_obj == True:         
                # == object detection ===============================
                global_npc_info, local_npc_info, global_ped_info, local_ped_info, global_obs_info, local_obs_info = self.forward_object_detector.detect_object(self.status_msg, self.object_data)                                                
                
                global_info_pub = global_npc_info, global_ped_info, global_obs_info
                local_info_pub = local_npc_info, local_ped_info, local_obs_info
                if self.object_num > 0:
                    self.global_info_pub.publish(global_info_pub)
                print("====", local_npc_info, local_ped_info, local_obs_info)       
                # ================================
                # self.print_info()
            rate.sleep()
 
    #=======================================================================
    # object Subscribe
    def object_callback(self, msg):
        self.is_obj = True
        self.object_data = msg

        self.object_num = msg.num_of_npcs + msg.num_of_obstacle + msg.num_of_pedestrian
  
    # Traffic light Subscribe
    def getTL_callback(self, msg):
        self.is_traffic = True
        self.tl_msg = GetTrafficLightStatus()
        self.tl_msg = msg
    
    # Ego-vehicle Status Subscribe
    def ego_status_callback(self, msg):
        self.status_msg = msg
        self.is_status = True

# =======================================
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


if __name__ == '__main__':
    # try:
    perception()
    # except rospy.ROSInterruptException:
    #     pass
