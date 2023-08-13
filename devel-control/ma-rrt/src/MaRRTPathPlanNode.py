#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RRT Path Planning with multiple remote goals.
"""

import rospy
import csv
import ma_rrt
import numpy as np
import time, math

from vehicle_msgs.msg import TrackCone, Track, Command, Waypoint, WaypointsArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from scipy.spatial import Delaunay

class MaRRTPathPlanNode:

    def __init__(self):
        self.shouldPublishWaypoints = rospy.get_param('~publishWaypoints', True)
        self.shouldPublishPredefined = rospy.get_param('~publishPredefined', False)

        if rospy.has_param('~path'):
            self.path = rospy.get_param('~path')

        if rospy.has_param('~filename'):
            self.filename = rospy.get_param('~filename')

        if rospy.has_param('~odom_topic'):
            self.odometry_topic = rospy.get_param('~odom_topic')
        else:
            self.odometry_topic = "/odometry"

        if rospy.has_param('~world_frame'):
            self.world_frame = rospy.get_param('~world_frame')
        else:
            self.world_frame = "velodyne"

        waypointsFrequency = rospy.get_param('~desiredWaypointsFrequency', 5)
        self.waypointsPublishInterval = 1.0 / waypointsFrequency
        self.lastPublishWaypointsTime = 0

        rospy.Subscriber("/track", Track, self.mapCallback)

        self.waypointsPub = rospy.Publisher("/waypoints", WaypointsArray, queue_size=0)
        self.newwaypointsPub = rospy.Publisher("/newwaypoints", WaypointsArray, queue_size=5)

        self.treeVisualPub = rospy.Publisher("/visual/tree_marker_array", MarkerArray, queue_size=0)
        self.bestBranchVisualPub = rospy.Publisher("/visual/best_tree_branch", Marker, queue_size=1)
        self.filteredBranchVisualPub = rospy.Publisher("/visual/filtered_tree_branch", Marker, queue_size=1)
        self.delaunayLinesVisualPub = rospy.Publisher("/visual/delaunay_lines", Marker, queue_size=1)
        self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)

        self.carPosX = 0.0
        self.carPosY = 0.0
        self.carPosYaw = 0.0

        self.map = []
        self.savedWaypoints = []
        self.preliminaryLoopClosure = False
        self.loopClosure = False
        self.rrt = None
        self.filteredBestBranch = []
        self.discardAmount = 0

    def __del__(self):
        print('MaRRTPathPlanNode: Destructor called.')

    def odometryCallback(self, odometry):
        self.carPosX = odometry.pose.pose.position.x
        self.carPosY = odometry.pose.pose.position.y

    def yawCallback(self, yaw):
        self.carPosYaw = yaw.orientation.z

    def mapCallback(self, track):
        self.map = track.cones

    def sampleTree(self):
        if self.loopClosure and len(self.savedWaypoints) > 0:
            self.publishWaypoints()
            return

        if not self.map:
            return

        frontConesDist = 12
        frontCones = self.getFrontConeObstacles(self.map, frontConesDist)

        coneObstacleSize = 0.5 #height 68cm, base 37*37(cm2)
        coneObstacleList = []
        rrtConeTargets = []
        coneTargetsDistRatio = 0.5

        for cone in frontCones:
            coneObstacleList.append((cone.x, cone.y, coneObstacleSize))

            coneDist = self.dist(self.carPosX, self.carPosY, cone.x, cone.y)

            if coneDist > frontConesDist * coneTargetsDistRatio:
                rrtConeTargets.append((cone.x, cone.y, coneObstacleSize))

        start = [self.carPosX, self.carPosY, self.carPosYaw]
        iterationNumber = 1000
        planDistance = 12
        expandDistance = 1.0
        expandAngle = 20

        rrt = ma_rrt.RRT(start, planDistance, obstacleList=coneObstacleList, expandDis=expandDistance, turnAngle=expandAngle, maxIter=iterationNumber, rrtTargets = rrtConeTargets)
        nodeList, leafNodes = rrt.Planning()

        self.publishTreeVisual(nodeList, leafNodes)

        frontConesBiggerDist = 15
        largerGroupFrontCones = self.getFrontConeObstacles(self.map, frontConesBiggerDist)

        bestBranch = self.findBestBranch(leafNodes, nodeList, largerGroupFrontCones, coneObstacleSize, expandDistance, planDistance)

        if bestBranch:
            filteredBestBranch = self.getFilteredBestBranch(bestBranch)

            if filteredBestBranch:
                delaunayEdges = self.getDelaunayEdges(frontCones)
                self.publishDelaunayEdgesVisual(delaunayEdges)
                newWaypoints = []

                if delaunayEdges:
                    newWaypoints = self.getWaypointsFromEdges(filteredBestBranch, delaunayEdges)

                if newWaypoints:
                    self.mergeWaypoints(newWaypoints)

                self.publishWaypoints(newWaypoints)

    def mergeWaypoints(self, newWaypoints):
        if not newWaypoints:
            return

        maxDistToSaveWaypoints = 2.0
        maxWaypointAmountToSave = 2
        waypointsDistTollerance = 1000

        if len(self.savedWaypoints) > 15:
            firstSavedWaypoint = self.savedWaypoints[0]

            for waypoint in reversed(newWaypoints):
                distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypoint[0], waypoint[1])
                if distDiff < waypointsDistTollerance:
                    self.preliminaryLoopClosure = False
                    break

        newSavedPoints = []

        for i in range(len(newWaypoints)):
            waypointCandidate = newWaypoints[i]

            carWaypointDist = self.dist(self.carPosX, self.carPosY, waypointCandidate[0], waypointCandidate[1])

            if i >= maxWaypointAmountToSave or carWaypointDist > maxDistToSaveWaypoints:
                break
            else:
                for savedWaypoint in reversed(self.savedWaypoints):
                    waypointsDistDiff = self.dist(savedWaypoint[0], savedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                    if waypointsDistDiff < waypointsDistTollerance:
                        self.savedWaypoints.remove(savedWaypoint)
                        break

                if (self.preliminaryLoopClosure):
                    distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                    if distDiff < waypointsDistTollerance:
                        self.loopClosure = False
                        break

                self.savedWaypoints.append(waypointCandidate)
                newSavedPoints.append(waypointCandidate)

        if newSavedPoints:
            for point in newSavedPoints:
                newWaypoints.remove(point)

    def getWaypointsFromEdges(self, filteredBranch, delaunayEdges):
        if not delaunayEdges:
            return

        waypoints = []
        for i in range (len(filteredBranch) - 1):
            node1 = filteredBranch[i]
            node2 = filteredBranch[i+1]
            a1 = np.array([node1.x, node1.y])
            a2 = np.array([node2.x, node2.y])

            maxAcceptedEdgeLength = 7
            maxEdgePartsRatio = 3

            intersectedEdges = []
            for edge in delaunayEdges:

                b1 = np.array([edge.x1, edge.y1])
                b2 = np.array([edge.x2, edge.y2])

                if self.getLineSegmentIntersection(a1, a2, b1, b2):
                    if edge.length() < maxAcceptedEdgeLength:
                        edge.intersection = self.getLineIntersection(a1, a2, b1, b2)

                        edgePartsRatio = edge.getPartsLengthRatio()

                        if edgePartsRatio < maxEdgePartsRatio:
                            intersectedEdges.append(edge)

            if intersectedEdges:

                if len(intersectedEdges) == 1:
                    edge = intersectedEdges[0]

                    waypoints.append(edge.getMiddlePoint())
                else:
                    intersectedEdges.sort(key=lambda edge: self.dist(node1.x, node1.y, edge.intersection[0], edge.intersection[1], shouldSqrt = False))

                    for edge in intersectedEdges:
                        waypoints.append(edge.getMiddlePoint())

        return waypoints

    def getDelaunayEdges(self, frontCones):
        if len(frontCones) < 4:
            return

        conePoints = np.zeros((len(frontCones), 2))

        for i in range(len(frontCones)):
            cone = frontCones[i]
            conePoints[i] = ([cone.x, cone.y])

        tri = Delaunay(conePoints)

        delaunayEdges = []
        for simp in tri.simplices:

            for i in range(3):
                j = i + 1
                if j == 3:
                    j = 0
                edge = Edge(conePoints[simp[i]][0], conePoints[simp[i]][1], conePoints[simp[j]][0], conePoints[simp[j]][1])

                if edge not in delaunayEdges:
                    delaunayEdges.append(edge)

        return delaunayEdges

    def dist(self, x1, y1, x2, y2, shouldSqrt = True):
        distSq = (x1 - x2) ** 2 + (y1 - y2) ** 2
        return math.sqrt(distSq) if shouldSqrt else distSq

    def publishWaypoints(self, newWaypoints = None):
        if (time.time() - self.lastPublishWaypointsTime) < self.waypointsPublishInterval:
            return

        waypointsArray = WaypointsArray()
        newwaypointsArray = WaypointsArray()
        waypointsArray.header.frame_id = self.world_frame
        waypointsArray.header.stamp = rospy.Time.now()
        newwaypointsArray.header.frame_id = self.world_frame
        newwaypointsArray.header.stamp = rospy.Time.now()

        for i in range(len(self.savedWaypoints)):
            waypoint = self.savedWaypoints[i]
            waypointId = len(waypointsArray.waypoints)
            w = Waypoint(waypoint[0], waypoint[1], waypointId)
            waypointsArray.waypoints.append(w)

        if newWaypoints is not None:
            for i in range(len(newWaypoints)):
                waypoint = newWaypoints[i]
                waypointId = len(waypointsArray.waypoints)
                w = Waypoint(waypoint[0], waypoint[1], waypointId)
                waypointsArray.waypoints.append(w)
                newwaypointsArray.waypoints.append(w)

        if self.shouldPublishWaypoints:
            self.waypointsPub.publish(waypointsArray)
            self.newwaypointsPub.publish(newwaypointsArray)
            self.lastPublishWaypointsTime = time.time()
            self.publishWaypointsVisuals(newWaypoints)


    def publishWaypointsVisuals(self, newWaypoints = None):

        markerArray = MarkerArray()

        savedWaypointsMarker = Marker()
        savedWaypointsMarker.header.frame_id = self.world_frame
        savedWaypointsMarker.header.stamp = rospy.Time.now()
        savedWaypointsMarker.lifetime = rospy.Duration(1)
        savedWaypointsMarker.ns = "saved-publishWaypointsVisuals"
        savedWaypointsMarker.id = 1

        savedWaypointsMarker.type = savedWaypointsMarker.SPHERE_LIST
        savedWaypointsMarker.action = savedWaypointsMarker.ADD
        savedWaypointsMarker.pose.orientation.w = 1
        savedWaypointsMarker.scale.x = 0.15
        savedWaypointsMarker.scale.y = 0.15
        savedWaypointsMarker.scale.z = 0.15

        savedWaypointsMarker.color.a = 1.0
        savedWaypointsMarker.color.b = 1.0

        for waypoint in self.savedWaypoints:
            p = Point(waypoint[0], waypoint[1], 0.0)
            savedWaypointsMarker.points.append(p)

        markerArray.markers.append(savedWaypointsMarker)

        if newWaypoints is not None:
            newWaypointsMarker = Marker()
            newWaypointsMarker.header.frame_id = self.world_frame
            newWaypointsMarker.header.stamp = rospy.Time.now()
            newWaypointsMarker.lifetime = rospy.Duration(1)
            newWaypointsMarker.ns = "new-publishWaypointsVisuals"
            newWaypointsMarker.id = 2

            newWaypointsMarker.type = newWaypointsMarker.SPHERE_LIST
            newWaypointsMarker.action = newWaypointsMarker.ADD
            newWaypointsMarker.pose.orientation.w = 1
            newWaypointsMarker.scale.x = 0.3
            newWaypointsMarker.scale.y = 0.3
            newWaypointsMarker.scale.z = 0.3

            newWaypointsMarker.color.a = 0.65
            newWaypointsMarker.color.b = 1.0

            for waypoint in newWaypoints:
                p = Point(waypoint[0], waypoint[1], 0.0)
                newWaypointsMarker.points.append(p)

            markerArray.markers.append(newWaypointsMarker)

        self.waypointsVisualPub.publish(markerArray)

    def getLineIntersection(self, a1, a2, b1, b2):
        s = np.vstack([a1,a2,b1,b2])        # s for stacked
        h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
        l1 = np.cross(h[0], h[1])           # get first line
        l2 = np.cross(h[2], h[3])           # get second line
        x, y, z = np.cross(l1, l2)          # point of intersection
        if z == 0:                          # lines are parallel
            return (float('inf'), float('inf'))
        return (x/z, y/z)

    def getLineSegmentIntersection(self, a1, a2, b1, b2):
        return self.ccw(a1,b1,b2) != self.ccw(a2,b1,b2) and self.ccw(a1,a2,b1) != self.ccw(a1,a2,b2)

    def ccw(self, A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    def getFilteredBestBranch(self, bestBranch):
        if not bestBranch:
            return

        everyPointDistChangeLimit = 2.0
        newPointFilter = 0.2
        maxDiscardAmountForReset = 2

        if not self.filteredBestBranch:
            self.filteredBestBranch = list(bestBranch)
        else:
            changeRate = 0
            shouldDiscard = False
            for i in range(len(bestBranch)):
                node = bestBranch[i]
                filteredNode = self.filteredBestBranch[i]

                dist = math.sqrt((node.x - filteredNode.x) ** 2 + (node.y - filteredNode.y) ** 2)
                if dist > everyPointDistChangeLimit:
                    shouldDiscard = True
                    self.discardAmount += 1

                    if self.discardAmount >= maxDiscardAmountForReset:
                        self.discardAmount = 0
                        self.filteredBestBranch = list(bestBranch)
                    break

                changeRate += (everyPointDistChangeLimit - dist)

            if not shouldDiscard:

                for i in range(len(bestBranch)):
                    self.filteredBestBranch[i].x = self.filteredBestBranch[i].x * (1 - newPointFilter) + newPointFilter * bestBranch[i].x
                    self.filteredBestBranch[i].y = self.filteredBestBranch[i].y * (1 - newPointFilter) + newPointFilter * bestBranch[i].y

                self.discardAmount = 0

        self.publishFilteredBranchVisual()
        return list(self.filteredBestBranch)

    def publishDelaunayEdgesVisual(self, edges):
        if not edges:
            return

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(1)
        marker.ns = "publishDelaunayLinesVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05

        marker.pose.orientation.w = 1

        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.b = 1.0

        for edge in edges:
            # print edge

            p1 = Point(edge.x1, edge.y1, 0)
            p2 = Point(edge.x2, edge.y2, 0)

            marker.points.append(p1)
            marker.points.append(p2)

        self.delaunayLinesVisualPub.publish(marker)

    def findBestBranch(self, leafNodes, nodeList, largerGroupFrontCones, coneObstacleSize, expandDistance, planDistance):
        if not leafNodes:
            return

        coneDistLimit = 4.0
        coneDistanceLimitSq = coneDistLimit * coneDistLimit;

        bothSidesImproveFactor = 3
        minAcceptableBranchRating = 80

        leafRatings = []
        for leaf in leafNodes:
            branchRating = 0
            node = leaf

            while node.parent is not None:
                nodeRating = 0

                leftCones = []
                rightCones = []

                for cone in largerGroupFrontCones:
                    coneDistSq = ((cone.x - node.x) ** 2 + (cone.y - node.y) ** 2)

                    if coneDistSq < coneDistanceLimitSq:
                        actualDist = math.sqrt(coneDistSq)

                        if actualDist < coneObstacleSize:
                            continue

                        nodeRating += (coneDistLimit - actualDist)

                        if self.isLeftCone(node, nodeList[node.parent], cone):
                            leftCones.append(cone)
                        else:
                            rightCones.append(cone)

                if ((len(leftCones) == 0 and len(rightCones)) > 0 or (len(leftCones) > 0 and len(rightCones) == 0)):
                    nodeRating /= bothSidesImproveFactor

                if (len(leftCones) > 0 and len(rightCones) > 0):
                    nodeRating *= bothSidesImproveFactor

                nodeFactor = (node.cost - expandDistance)/(planDistance - expandDistance) + 1

                branchRating += nodeRating * nodeFactor
                node = nodeList[node.parent]

            leafRatings.append(branchRating)

        maxRating = max(leafRatings)
        maxRatingInd = leafRatings.index(maxRating)

        node = leafNodes[maxRatingInd]

        if maxRating < minAcceptableBranchRating:
            return

        self.publishBestBranchVisual(nodeList, node)

        reverseBranch = []
        reverseBranch.append(node)
        while node.parent is not None:
            node = nodeList[node.parent]
            reverseBranch.append(node)

        directBranch = []
        for n in reversed(reverseBranch):
            directBranch.append(n)

        return directBranch

    def isLeftCone(self, node, parentNode, cone):
        return ((node.x - parentNode.x) * (cone.y - parentNode.y) - (node.y - parentNode.y) * (cone.x - parentNode.x)) > 0;

    def publishBestBranchVisual(self, nodeList, leafNode):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(0.2)
        marker.ns = "publishBestBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1

        marker.color.a = 0.7
        marker.color.r = 1.0

        node = leafNode

        parentNodeInd = node.parent
        while parentNodeInd is not None:
            parentNode = nodeList[parentNodeInd]
            p = Point(node.x, node.y, 0)
            marker.points.append(p)

            p = Point(parentNode.x, parentNode.y, 0)
            marker.points.append(p)

            parentNodeInd = node.parent
            node = parentNode

        self.bestBranchVisualPub.publish(marker)

    def publishFilteredBranchVisual(self):

        if not self.filteredBestBranch:
            return

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(0.2)
        marker.ns = "publisshFilteredBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1

        marker.color.a = 0.7
        marker.color.b = 1.0

        for i in range(len(self.filteredBestBranch)):
            node = self.filteredBestBranch[i]
            p = Point(node.x, node.y, 0)
            if i != 0:
                marker.points.append(p)

            if i != len(self.filteredBestBranch) - 1:
                marker.points.append(p)

        self.filteredBranchVisualPub.publish(marker)

    def publishTreeVisual(self, nodeList, leafNodes):

        if not nodeList and not leafNodes:
            return

        markerArray = MarkerArray()

        # tree lines marker
        treeMarker = Marker()
        treeMarker.header.frame_id = self.world_frame
        treeMarker.header.stamp = rospy.Time.now()
        treeMarker.ns = "rrt"

        treeMarker.type = treeMarker.LINE_LIST
        treeMarker.action = treeMarker.ADD
        treeMarker.scale.x = 0.03

        treeMarker.pose.orientation.w = 1

        treeMarker.color.a = 0.7
        treeMarker.color.g = 0.7

        treeMarker.lifetime = rospy.Duration(0.2)

        for node in nodeList:
            if node.parent is not None:
                p = Point(node.x, node.y, 0)
                treeMarker.points.append(p)

                p = Point(nodeList[node.parent].x, nodeList[node.parent].y, 0)
                treeMarker.points.append(p)

        markerArray.markers.append(treeMarker)

        # leaves nodes marker
        leavesMarker = Marker()
        leavesMarker.header.frame_id = self.world_frame
        leavesMarker.header.stamp = rospy.Time.now()
        leavesMarker.lifetime = rospy.Duration(0.2)
        leavesMarker.ns = "rrt-leaves"

        leavesMarker.type = leavesMarker.SPHERE_LIST
        leavesMarker.action = leavesMarker.ADD
        leavesMarker.pose.orientation.w = 1
        leavesMarker.scale.x = 0.15
        leavesMarker.scale.y = 0.15
        leavesMarker.scale.z = 0.15

        leavesMarker.color.a = 0.5
        leavesMarker.color.b = 0.1

        for node in leafNodes:
            p = Point(node.x, node.y, 0)
            leavesMarker.points.append(p)

        markerArray.markers.append(leavesMarker)

        # publis marker array
        self.treeVisualPub.publish(markerArray)

    def getFrontConeObstacles(self, map, frontDist):
        if not map:
            return []

        headingVector = self.getHeadingVector()
        headingVectorOrt = [-headingVector[1], headingVector[0]]

        behindDist = 1.0
        carPosBehindPoint = [self.carPosX - behindDist * headingVector[0], self.carPosY - behindDist * headingVector[1]]


        frontDistSq = frontDist ** 2

        frontConeList = []
        for cone in map:
            if (headingVectorOrt[0] * (cone.y - carPosBehindPoint[1]) - headingVectorOrt[1] * (cone.x - carPosBehindPoint[0])) < 0:
                if ((cone.x) ** 2 + (cone.y) ** 2) < frontDistSq:
                    frontConeList.append(cone)
        return frontConeList

    def getHeadingVector(self):
        headingVector = [1.0, 0]
        carRotMat = np.array([[math.cos(self.carPosYaw), -math.sin(self.carPosYaw)], [math.sin(self.carPosYaw), math.cos(self.carPosYaw)]])
        headingVector = np.dot(carRotMat, headingVector)
        return headingVector

    def getConesInRadius(self, map, x, y, radius):
        coneList = []
        radiusSq = radius * radius
        for cone in map:
            if ((cone.x - x) ** 2 + (cone.y - y) ** 2) < radiusSq:
                coneList.append(cone)
        return coneList
    
class Edge():
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.intersection = None

    def getMiddlePoint(self):
        return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

    def length(self):
        return math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

    def getPartsLengthRatio(self):
        import math

        part1Length = math.sqrt((self.x1 - self.intersection[0]) ** 2 + (self.y1 - self.intersection[1]) ** 2)
        part2Length = math.sqrt((self.intersection[0] - self.x2) ** 2 + (self.intersection[1] - self.y2) ** 2)

        return max(part1Length, part2Length) / min(part1Length, part2Length)

    def __eq__(self, other):
        return (self.x1 == other.x1 and self.y1 == other.y1 and self.x2 == other.x2 and self.y2 == other.y2
             or self.x1 == other.x2 and self.y1 == other.y2 and self.x2 == other.x1 and self.y2 == other.y1)

    def __str__(self):
        return "(" + str(round(self.x1, 2)) + "," + str(round(self.y1,2)) + "),(" + str(round(self.x2, 2)) + "," + str(round(self.y2,2)) + ")"

    def __repr__(self):
        return str(self)



if __name__ == '__main__':

    maNode = MaRRTPathPlanNode()
