// #include <iostream>
// #include "ros/ros.h"
// #include <cmath>

// #include "./localization/point.h"
// #include "./localization/vehicle_state.h"
// #include "tf/transform_broadcaster.h"

// #include <geometry_msgs/Pose2D.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/Vector3.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Quaternion.h>
// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/Pose.h>

// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <morai_msgs/EgoVehicleStatus.h>
// #include <morai_msgs/CtrlCmd.h>

// #include "std_msgs/String.h"
// #include <sstream>

// #include <ros/package.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <math.h>
// #include <vector>
// #include <fstream>
// #include <sstream>

// class Reader {
// public:
//     Reader() : loop_rate(10)
//     {
//         ros::NodeHandle n1;

//         // Create the publisher for the control commands
//         // ctrl_cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);

//         // Create the publisher for the path messages
//         global_path_pub = n1.advertise<nav_msgs::Path>("/global_path", 1);
//         // local_path_pub = nh.advertise<nav_msgs::Path>("/local_path", 1);

//         // Read the path from file
//         readPathFromFile();
//     }
    
    

//     void readPathFromFile()
//     {
//         // Path to the file you want to read (change this path as needed)
//         std::string file_path = "/home/rml-phat/catkin_ws/src/MORAI/erp_cpp/path/lidar_test.txt";

//         // Open the file
//         std::ifstream file(file_path);
//         if (!file.is_open())
//         {
//             ROS_ERROR_STREAM("Failed to open file: " << file_path);
//             return;
//         }

//         // Read the file line by line and process the data
//         std::string line;
//         while (std::getline(file, line))
//         {
//             // Assuming each line in the file contains a point in the format "x y theta velocity"
//             double x, y, z;
//             std::istringstream iss(line);
//             global_path.header.frame_id = "/map";
//             if (!(iss >> x >> y >> z))
//             {
//                 ROS_WARN_STREAM("Skipped invalid line in the file: " << line);
//                 continue;
//             }

//             // Create a geometry_msgs::PoseStamped message to represent each point
//             geometry_msgs::PoseStamped pose;
//             pose.pose.position.x = x;
//             pose.pose.position.y = y;
//             pose.pose.position.z = z;
//             pose.pose.orientation.x = 0;
//             pose.pose.orientation.y = 0;
//             pose.pose.orientation.z = 0;
//             pose.pose.orientation.w = 1;

//             // Add the pose to the global path
//             global_path.poses.push_back(pose);
//         }

//         // Close the file after reading
//         file.close();
//     }

//     void run()
//     {
        
//         while (ros::ok())
//         {
//             // Publish the global path
//             global_path_pub.publish(global_path);
//             std::cout << global_path << std::endl;
            

//             // Sleep to control the publishing rate
//             loop_rate.sleep();
//         }
//     }

// private:
//     ros::Publisher global_path_pub;
//     nav_msgs::Path global_path;
//     ros::Rate loop_rate;
// };


// int main(int argc, char **argv)
// {

//   ros::init(argc, argv, "Planner");

//   ros::NodeHandle nh;
//   Reader reader;
//   reader.run();
//   return 0;
// }

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>

nav_msgs::Path read_txt(const std::string& file_path)
{
    std::string full_file_name = file_path;
    std::ifstream inFile(full_file_name);
    if (!inFile.is_open())
    {
        ROS_ERROR_STREAM("Failed to open file: " << full_file_name);
        return nav_msgs::Path();
    }

    nav_msgs::Path out_path;
    out_path.header.frame_id = "/map";

    std::string line;
    while (std::getline(inFile, line))
    {
        std::istringstream iss(line);
        double x, y, z;
        if (!(iss >> x >> y >> z))
        {
            ROS_WARN_STREAM("Invalid line in file: " << line);
            continue;
        }

        geometry_msgs::PoseStamped read_pose;
        read_pose.pose.position.x = x;
        read_pose.pose.position.y = y;
        read_pose.pose.position.z = z;
        read_pose.pose.orientation.x = 0;
        read_pose.pose.orientation.y = 0;
        read_pose.pose.orientation.z = 0;
        read_pose.pose.orientation.w = 1;
        out_path.poses.push_back(read_pose);
    }

    inFile.close();
    return out_path;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;

    // Read the path from "path.txt" file
    std::string file_path = "/home/rml-phat/catkin_ws/src/MORAI/erp_cpp/path/multi-lane21.txt"; // Replace with the name of your 
    nav_msgs::Path path_msg = read_txt(file_path);

    // Create the publisher for the path
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/global_path", 1000);

    // Publish the path at 10 Hz
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        path_pub.publish(path_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
