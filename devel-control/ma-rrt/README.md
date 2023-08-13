# RRT with multiple remote goals.

### Notes
- The algorithm does not utilize the cones' color information, but instead a simple logic is used, which rewards the branches with cones from both sides (see findBestBranch(...) method), and penalizes the branches having cones only from one side. With cone classification information a better rating system can be implemented and applied.


## Inputs, outputs, params

#### Inputs
- rospy.Subscriber("/map", Map, ...)
- rospy.Subscriber("/odometry", Odometry, ...)
- rospy.Subscriber("/track", Track, ...)
- rospy.Subscriber("/newwaypoints", WaypointsArray, ...)

#### Outputs
- rospy.Publisher("/waypoints", WaypointsArray, ...)

#### Outputs (Visuals)
- rospy.Publisher("/visual/tree_marker_array", MarkerArray, ...)
- rospy.Publisher("/visual/best_tree_branch", Marker, ...)
- rospy.Publisher("/visual/filtered_tree_branch", Marker, ...)
- rospy.Publisher("/visual/delaunay_lines", Marker, ...)
- rospy.Publisher("/visual/waypoints", MarkerArray, ...)

#### Parameters to tune (main)
- planDistance = 12 m - Maximum length of tree branch
- expandDistance = 1 m - Length of tree node/step
- expandAngle = 20 deg - constraining angle for next tree nodes
- coneObstacleSize = 1.1 m - size of obstacles derived from cone position
- coneTargetsDistRatio = 0.5 - ratio to frontConesDist for deriving remote cone goals

