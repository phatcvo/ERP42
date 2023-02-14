# 2023.01.15
# Phat C. Vo in RML

# -*- coding: utf-8 -*-
import sys
import os
import copy
from math import cos, sin, sqrt, pow, atan2, pi
import numpy as np

# dijkstra algorithm =================================================================
class Dijkstra:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links
        self.weight = self.get_weight_matrix()
        self.solution = {'node_path':[], 'link_path':[], 'point_path':[]} # solution
        self.lane_change_link_idx = []


    def get_weight_matrix(self):
        # initial setup
        weight = dict() 
        for from_node_id, from_node in self.nodes.items():
            # All weights going from the current node to other nodes
            weight_from_this_node = dict()
            for to_node_id, to_node in self.nodes.items():
                weight_from_this_node[to_node_id] = float('inf')
            # Add to the full weight matrix
            weight[from_node_id] = weight_from_this_node

        for from_node_id, from_node in self.nodes.items():
            # From current node to current node "cost = 0"
            weight[from_node_id][from_node_id] = 0

            for to_node in from_node.get_to_nodes():
                # Find links from the current node to "to_node", and find the fastest link among them.
                shortest_link, min_cost = from_node.find_shortest_link_leading_to_node(to_node)
                weight[from_node_id][to_node.idx] = min_cost           

        return weight      
        

    def find_nearest_node_idx(self, distance, s):
        
        idx_list = self.nodes.keys()
        min_value = float('inf')

        if sys.version_info[0] >= 3: # for python3
            # In python3, dict.keys() is of type dict_keys and is not subscriptable.
            # Therefore, you must access it by index after changing it to a list.
            idx_list_subscriptable = list(idx_list)
            # NOTE: very important Not a meaningless value. 
            # If None is given, it malfunctions.
            min_idx = idx_list_subscriptable[-1] 
        else: # for python2
            min_idx = idx_list[-1]

        for idx in idx_list:
            # if distance[idx] < min_value:
            #     print('distance[idx]: {0},s[idx]{1}, min value {2}',distance[idx],s[idx],min_value)

            if distance[idx] < min_value and s[idx] == False :
                min_value = distance[idx]
                min_idx = idx
        return min_idx


    def find_shortest_path(self, start_node_idx, end_node_idx): 
        # [STEP #0] reset
        # s reset         >> s = [False] * len(self.nodes)
        # from_node reset >> from_node = [start_node_idx] * len(self.nodes)
        s = dict()
        from_node = dict() 
        for node_id in self.nodes.keys():
            s[node_id] = False
            from_node[node_id] = start_node_idx
        
        
        s[start_node_idx] = True
        distance =copy.deepcopy(self.weight[start_node_idx])

        # [STEP #1] Dijkstra core code
        for i in range(len(self.nodes.keys()) - 1):
            selected_node_idx = self.find_nearest_node_idx(distance, s)
            s[selected_node_idx] = True
            
            for j, to_node_idx in enumerate(self.nodes.keys()):
                if s[to_node_idx] == False:
                    distance_candidate = distance[selected_node_idx] + self.weight[selected_node_idx][to_node_idx]
                    if distance_candidate < distance[to_node_idx]:
                        distance[to_node_idx] = distance_candidate
                        from_node[to_node_idx] = selected_node_idx
 


        # [STEP #2] Create node path
        tracking_idx = end_node_idx
        node_path = [end_node_idx]
        
        while start_node_idx != tracking_idx:
            tracking_idx = from_node[tracking_idx]
            node_path.append(tracking_idx)     

        node_path.reverse()

        # [STEP #3] Generate link path
        link_path = []
        for i in range(len(node_path) - 1):
            from_node_idx = node_path[i]
            to_node_idx = node_path[i + 1]

            from_node = self.nodes[from_node_idx]
            to_node = self.nodes[to_node_idx]

            shortest_link, min_cost = from_node.find_shortest_link_leading_to_node(to_node)
            link_path.append(shortest_link.idx)
        # Result judgment
        if len(link_path) == 0:
            return False, {'node_path': node_path, 'link_path':link_path, 'point_path':[]}


        # [STEP #4] Create point path
        point_path = []
    
        # [USER OPTION] Create a path to change lanes and reflect it to point_path
        user_option_draw_lane_change = True
        
        for link_id in link_path:
            link = self.links[link_id]
            if link.is_it_for_lane_change() and user_option_draw_lane_change:  
                # If it is a lane change link and the lane change is to cross multiple lanes, 
                # calculate the point by considering it as a lane change at once
                lane_ch_pair_list = link.get_lane_change_pair_list()
                lane_ch_first = lane_ch_pair_list[0]
                lane_ch_last = lane_ch_pair_list[-1]
                lane_ch_distance = 20 * link.get_number_of_lane_change()

                output_path = self.draw_lane_change(
                    lane_ch_first['from'], lane_ch_last['to'], lane_ch_distance, 1)
        
                output_path = np.array(output_path)
                x = output_path[:,0]
                y = output_path[:,1]
                for i in range(len(x)):
                    point_path.append([x[i], y[i], 0])

            else:
                # Not a lane change link
                for point in link.points:
                    point_path.append([point[0], point[1], 0])

        return True, {'node_path': node_path, 'link_path':link_path, 'point_path':point_path}
    

    def draw_lane_change(self, start_link, end_link, lane_change_distance, step_size):
        output_path = []


        translation = [start_link.points[0][0], start_link.points[0][1]]
        theta = atan2(start_link.points[1][1] - start_link.points[0][1], start_link.points[1][0] - start_link.points[0][0])

        t = np.array([
                    [cos(theta), -sin(theta),translation[0]],
                    [sin(theta),  cos(theta),translation[1]],
                    [0         ,  0         ,1            ]])

        det_t = np.array([
                        [t[0][0], t[1][0], -(t[0][0]*translation[0] + t[1][0]*translation[1])],
                        [t[0][1], t[1][1], -(t[0][1]*translation[0] + t[1][1]*translation[1])],
                        [0      , 0      , 1                                                 ]])

        world_end_link_list = []
        for point in end_link.points :
            world_end_link_list.append([point[0], point[1], 1])

        world_end_link_matrix = np.array(world_end_link_list).T
        local_end_link_matrix = det_t.dot(world_end_link_matrix).T

        min_dis=float('inf')
        local_end_point_list = []

        
        for point in local_end_link_matrix:
            if point[0] > 0:
                dis = abs(sqrt(point[0]*point[0] + point[1]*point[1]) - lane_change_distance)
                if dis < min_dis:
                    min_dis = dis
                    local_end_point_list = [[point[0]], [point[1]] ,[1]]
           
     
        local_end_point_matrix = np.array(local_end_point_list)
   
        
        x = []
        y = []
        x_interval = step_size
        xs = 0
        xf = local_end_point_matrix[0][0]

        ps=0.0
        pf=local_end_point_matrix[1][0]
        
        x_num = xf / x_interval
        for i in range(xs, int(x_num)): 
            x.append(i * x_interval)

        a = [0.0, 0.0, 0.0, 0.0]
        a[0] = ps
        a[1] = 0
        a[2] = 3.0 * (pf - ps) / (xf**2)
        a[3] = -2.0 * (pf - ps) / (xf**3)
        for i in x:
            result=a[3]*i**3 + a[2]*i**2 + a[1]*i + a[0]
            y.append(result)

        for i in range(0, len(y)) :
            local_change_path = np.array([[x[i]],[y[i]],[1]])
            global_change_path = t.dot(local_change_path)
            # print([global_change_path[0][0],global_change_path[1][0],0])
            output_path.append([global_change_path[0][0], global_change_path[1][0],0])


        end_point_index = 0
        for (i,end_point) in enumerate(local_end_link_matrix.tolist()):
            if end_point[0] == local_end_point_matrix[0][0] and end_point[1] == local_end_point_matrix[1][0] :
                end_point_index = i
                break
        

        for end_point in end_link.points[end_point_index:]:
            # print([end_point[0],end_point[1],0])
            output_path.append([end_point[0],end_point[1],0])

        return output_path

