#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import PointCloud
import numpy as np

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *
from shortest_path_algorithm import Dijkstra


class get_mgeo :

    def __init__(self):
        rospy.init_node('morai_mgeo_pub', anonymous=True)
        self.link_pub = rospy.Publisher('link',PointCloud, queue_size=1)
        self.node_pub = rospy.Publisher('node',PointCloud, queue_size=1)
        self.path_mgeo_pub = rospy.Publisher('/global_path',Path, queue_size=1)

        self.path_mgeo=Path()
        self.path_mgeo.header.frame_id='/map'

        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity'))
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes=node_set.nodes
        self.links=link_set.lines
        self.link_msg=self.getAllLinks()
        self.node_msg=self.getAllNode()
        print('# of nodes: ', len(node_set.nodes))
        print('# of links: ', len(link_set.lines))
        
        # PATH PLANNING ==================================================
        
        self.path_planner = Dijkstra(self.nodes, self.links)
        # is_find, self.path = self.path_planner.find_shortest_path('A119BS010332','A119BS010205')
        is_find, self.path = self.path_planner.find_shortest_path('A119BS010304','A119BS010337')
        
        # A119BS010210 school
        # self.path.append(self.path)
        print("Found shortest part: ", is_find)
        # print(self.path['node_path'])
        # print(self.path['link_path'])
        path_new = self.path['point_path']
        for i in range(len(path_new[:])):
            tmp = path_new[i]
            path_path = PoseStamped()
            path_path.pose.position.x = tmp[0]
            path_path.pose.position.y = tmp[1]
            path_path.pose.orientation.w = 1
            self.path_mgeo.poses.append(path_path)
            
        rate = rospy.Rate(1) 

        while not rospy.is_shutdown():
   
            self.link_pub.publish(self.link_msg)
            self.node_pub.publish(self.node_msg)
            self.path_mgeo_pub.publish(self.path_mgeo)
            # print(self.path_mgeo)
            rate.sleep()


    def getAllLinks(self):
        all_link=PointCloud()
        all_link.header.frame_id='map'
        
        for link_idx in self.links :
            for link_point in self.links[link_idx].points:
                tmp_point=Point32()
                tmp_point.x=link_point[0]
                tmp_point.y=link_point[1]
                tmp_point.z=link_point[2]
                all_link.points.append(tmp_point)

        return all_link
    
    def getAllNode(self):
        all_node=PointCloud()
        all_node.header.frame_id='map'
        for node_idx in self.nodes :
            tmp_point=Point32()
            tmp_point.x=self.nodes[node_idx].point[0]
            tmp_point.y=self.nodes[node_idx].point[1]
            tmp_point.z=self.nodes[node_idx].point[2]
            all_node.points.append(tmp_point)

        return all_node


if __name__ == '__main__':
    
    test_track=get_mgeo()


