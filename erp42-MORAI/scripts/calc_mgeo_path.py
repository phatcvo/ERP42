# 2023.01.15
# Phat C. Vo in RML

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd

import json
import io
import os
import sys

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)


from lib.mgeo.class_defs import *
from shortest_path_algorithm import Dijkstra

class mgeo_dijkstra_path :
    def __init__(self, map_name):

        current_path = os.path.dirname(os.path.realpath(__file__))
        sys.path.append(current_path)

        # load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/'+map_name))
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity'))
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes = node_set.nodes
        self.links = link_set.lines

        self.global_planner = Dijkstra(self.nodes,self.links)

    def calc_dijkstra_path(self, start_node, end_node):

        result, path = self.global_planner.find_shortest_path(start_node, end_node)
        self.x_list = []
        self.y_list = []
        self.z_list = []
        for waypoint in path["point_path"] :
            path_x = waypoint[0]
            path_y = waypoint[1]
            self.x_list.append(path_x)
            self.y_list.append(path_y)
            self.z_list.append(0.)

        path = pd.DataFrame({"x":self.x_list, "y":self.y_list, "z":self.z_list})

        path_data = path.apply(
            lambda point: Point(point["x"], point["y"]), axis=1
        ).tolist()

        return path_data

class Point(np.ndarray):
    def __new__(cls, x=0, y=0):
        obj = np.asarray((x, y), dtype=np.float64).view(cls)
        return obj

    @property
    def x(self):
        return self[0]

    @property
    def y(self):
        return self[1]

    @property
    def angle(self):
        return np.arctan2(self[1], self[0])

    def translate(self, dx=0, dy=0):
        return self + np.array((dx, dy), dtype=np.float64)

    def rotate(self, angle):
        rotation_matrix = np.array(((np.cos(angle), -np.sin(angle)), (np.sin(angle),  np.cos(angle))))
        return rotation_matrix.dot(self).view(Point)

    def distance(self, other=np.array([0, 0])):
        return np.linalg.norm(self - other)