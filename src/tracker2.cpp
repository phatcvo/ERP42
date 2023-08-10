#include <iostream>
#include "ros/ros.h"
#include <cmath>

#include "./localization/point.h"
#include "./localization/vehicle_state.h"
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>

#include "tf/transform_broadcaster.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class PurePursuit {
public:
    PurePursuit() {
        forward_point = geometry_msgs::Point();
        current_position = geometry_msgs::Point();
        is_look_forward_point = false;
        vehicle_length = 1.95;
        lfd = 5; // look ahead distance L
        kf = 0.95; // look forward gain
        min_lfd = 2;
        max_lfd = 30;
        steering = 0;
    }
    void getPath(const std::vector<geometry_msgs::PoseStamped>& msg) {
        path = msg;
    }
    
    void getEgoStatus(const morai_msgs::EgoVehicleStatus& msg) {
        current_vel = msg.velocity.x; // kph
        vehicle_yaw = (msg.heading) / 180.0 * M_PI; // rad
        current_position.x = msg.position.x;
        current_position.y = msg.position.y;
        current_position.z = msg.position.z;
    }
    double steeringAngle() {
        geometry_msgs::Point vehicle_position = current_position;
        is_look_forward_point = false;
        double vehicle_coordinate [3] = {0.0, 0.0, 0.0};

        // Rransformations of coordinates between the two coordinates
        std::vector<double> coor_origin = {vehicle_position.x, vehicle_position.y};
        std::vector<std::vector<double>> t = {
            {std::cos(vehicle_yaw), -std::sin(vehicle_yaw), coor_origin[0]},
            {std::sin(vehicle_yaw), std::cos(vehicle_yaw), coor_origin[1]},
            {0, 0, 1}
        };
        std::vector<std::vector<double>> det_t = {
            {t[0][0], t[1][0], -(t[0][0] * coor_origin[0] + t[1][0] * coor_origin[1])},
            {t[0][1], t[1][1], -(t[0][1] * coor_origin[0] + t[1][1] * coor_origin[1])},
            {0, 0, 1}
        };

        for (size_t i = 0; i < path.size(); ++i) {
            const auto& poseStamped = path[i];
            const geometry_msgs::Point& path_point = poseStamped.pose.position;
            std::vector<double> world_coordinate = {path_point.x, path_point.y, 1.0};
            // print Ego position
            std::cout << "Pos_Ego: (" << path_point.x << ", " << path_point.y << ", " << path_point.z << ")" << std::endl;
            // Convert map reference of world coordinate data to vehicle coordinate data
            vehicle_coordinate [0] = det_t[0][0] * world_coordinate[0] + det_t[0][1] * world_coordinate[1] + det_t[0][2] * world_coordinate[2];
            vehicle_coordinate [1] = det_t[1][0] * world_coordinate[0] + det_t[1][1] * world_coordinate[1] + det_t[1][2] * world_coordinate[2];
            vehicle_coordinate [2] = det_t[2][0] * world_coordinate[0] + det_t[2][1] * world_coordinate[1] + det_t[2][2] * world_coordinate[2];

            if (vehicle_coordinate [0] > 0) {
                double dis = std::sqrt(vehicle_coordinate [0] * vehicle_coordinate [0] + vehicle_coordinate [1] * vehicle_coordinate [1]);
                std::cout << "CTE:" << dis-lfd << std::endl;
                if (dis >= lfd) {
                    lfd = current_vel * kf;
                    if (lfd < min_lfd) {
                        lfd = min_lfd;
                    } else if (lfd > max_lfd) {
                        lfd = max_lfd;
                    }
                    forward_point = path_point;
                    is_look_forward_point = true;
                    break;
                }
            }
        }

        double theta = std::atan2(vehicle_coordinate [1], vehicle_coordinate [0]);
        if (is_look_forward_point) {
            steering = std::atan2(2 * vehicle_length * std::sin(theta), lfd) * 180 / M_PI; //deg
            std::cout << "Steering_point: " << steering << std::endl;
            return steering;
        } else {
            ROS_INFO("No forward point found.");
            return 0;
        }
        
    }
private:
    std::vector<geometry_msgs::PoseStamped> path;
    morai_msgs::EgoVehicleStatus statusMsg;

    geometry_msgs::Point forward_point;
    geometry_msgs::Point current_position;  
    geometry_msgs::Point rotated_point;
    bool is_look_forward_point;
    double vehicle_length;
    double lfd;
    double kf;
    double min_lfd;
    double max_lfd;
    double steering;
    double vehicle_yaw;
    double current_vel;
    // std::vector<double> vehicle_coordinate ;
};


class MainPlanner {
public:
    MainPlanner() : nh("~") {
        // Initialize ROS node and subscribers
        ros::Subscriber globalPathSub = nh.subscribe("/global_path", 1, &MainPlanner::globalPathCallback, this);
        ros::Subscriber egoStatusSub = nh.subscribe("/Ego_topic", 1, &MainPlanner::egoStatusCallback, this);
        
        // Initialize ROS publisher
        ctrlCmdPub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);
        
        // Other initializations
        isPath = true;
        isStatus = false;
        isGlobalPath = true;
        
        // Class import
        trackingControl = PurePursuit();
        
        // Global lattice_path
        ros::Rate rate(30); // 30Hz
        while (ros::ok()) {
            if (isStatus) {
                // == Path data read ==============================
                std::vector<geometry_msgs::PoseStamped> localPath;
                std::size_t currentWaypoint;
                std::tie(localPath, currentWaypoint) = getLocalPathCurrentWaypoint(globalPath, statusMsg);

                // Print the vector elements
                // for (const auto& pose : localPath) {
                //     std::cout << "Position: (" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << ")" << std::endl;
                // }
                // Assuming the local planner logic will be implemented here.
                // == Tracking control ===========================
                trackingControl.getPath(localPath);
                trackingControl.getEgoStatus(statusMsg);
                ctrlCmdMsg.steering = trackingControl.steeringAngle() / 180.0 * M_PI; //rad
                std::cout << "steering_pub_: " << ctrlCmdMsg.steering << std::endl;
                // ===============================================
                
                ctrlCmdMsg.accel = 1.0;
                ctrlCmdPub.publish(ctrlCmdMsg);
                std::cout << "=======" << std::endl;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
    
private:
    ros::NodeHandle nh;
    ros::Subscriber globalPathSub;
    ros::Subscriber localPathSub;
    ros::Subscriber egoStatusSub;
    ros::Publisher ctrlCmdPub;
    morai_msgs::CtrlCmd ctrlCmdMsg;
    bool isPath;
    bool isStatus;
    bool isGlobalPath;
    morai_msgs::EgoVehicleStatus statusMsg;
    std::vector<geometry_msgs::PoseStamped> globalPath;
    PurePursuit trackingControl;
    
    // Helper method to create current waypoint and local_path using global_path and vehicle status_msg
    std::tuple<std::vector<geometry_msgs::PoseStamped>, std::size_t> getLocalPathCurrentWaypoint(const std::vector<geometry_msgs::PoseStamped>& globalPath, const morai_msgs::EgoVehicleStatus& egoStatus) {
        std::vector<geometry_msgs::PoseStamped> localPath;
        double dist = 0.0;
        double currentX = egoStatus.position.x;
        double currentY = egoStatus.position.y;
        std::size_t currentWaypoint = 0;
        std::size_t localPathSize = 50;
        double minDist = std::numeric_limits<double>::infinity();
        
        for (std::size_t i = 0; i < globalPath.size(); ++i) {
            double dx = currentX - globalPath[i].pose.position.x;
            double dy = currentY - globalPath[i].pose.position.y;
            dist = sqrt(pow(dx, 2) + pow(dy, 2));
            if (minDist > dist) {
                minDist = dist;
                currentWaypoint = i;
            }
        }
        
        std::size_t lastLocalWaypoint = std::min(currentWaypoint + localPathSize, globalPath.size());
        
        for (std::size_t i = currentWaypoint; i < lastLocalWaypoint; ++i) {
            geometry_msgs::PoseStamped tmpPose;
            tmpPose.pose.position.x = globalPath[i].pose.position.x;
            tmpPose.pose.position.y = globalPath[i].pose.position.y;
            tmpPose.pose.position.z = globalPath[i].pose.position.z;
            tmpPose.pose.orientation.x = 0;
            tmpPose.pose.orientation.y = 0;
            tmpPose.pose.orientation.z = 0;
            tmpPose.pose.orientation.w = 1;
            localPath.push_back(tmpPose);
        }
        
        return std::make_tuple(localPath, currentWaypoint);
    }
    
    // Global path callback
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        globalPath = msg->poses;
        isGlobalPath = true;
    }
    
    // Ego-vehicle status callback
    void egoStatusCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
        statusMsg = *msg;
        isStatus = true;
        
        // Broadcast ego-vehicle's transform
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion quat;
        quat.setRPY(0, 0, statusMsg.heading / 180.0 * M_PI);
        transform.setOrigin(tf::Vector3(statusMsg.position.x, statusMsg.position.y, statusMsg.position.z));
        transform.setRotation(quat);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "gps"));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracker");
    MainPlanner mainPlanner;
    ros::spin();
    return 0;
}
