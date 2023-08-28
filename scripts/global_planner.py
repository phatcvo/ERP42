#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 2023.01.15
# Phat C. Vo in RML

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def read_txt(file_path):
    full_file_name = file_path
    try:
        with open(full_file_name, 'r') as inFile:
            lines = inFile.readlines()
    except IOError:
        rospy.logerr("Failed to open file: %s", full_file_name)
        return Path()

    out_path = Path()
    out_path.header.frame_id = "/map"

    for line in lines:
        parts = line.split()
        if len(parts) != 3:
            rospy.logwarn("Invalid line in file: %s", line)
            continue

        x, y, z = map(float, parts)
        read_pose = PoseStamped()
        read_pose.pose.position.x = x
        read_pose.pose.position.y = y
        read_pose.pose.position.z = z
        read_pose.pose.orientation.x = 0
        read_pose.pose.orientation.y = 0
        read_pose.pose.orientation.z = 0
        read_pose.pose.orientation.w = 1
        out_path.poses.append(read_pose)

    return out_path

def main():
    rospy.init_node("global_planner")
    file_path = "/home/rml-phat/catkin_ws/src/MORAI/erp_ros/path/kcity.txt"  # Replace with the path to your file
    path_msg = read_txt(file_path)

    path_pub = rospy.Publisher("/global_path", Path, queue_size=1000)
    rate = rospy.Rate(10)
    print ("======== global path ON =================")
    while not rospy.is_shutdown():
        path_pub.publish(path_msg)
        
        rate.sleep()

if __name__ == "__main__":
    main()