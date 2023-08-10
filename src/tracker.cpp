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
      // Convert heading from degrees to radians
        vehicle_yaw = (msg.heading) / 180.0 * M_PI;
        current_position.x = msg.position.x;
        current_position.y = msg.position.y;
        current_position.z = msg.position.z;
    }
    double steeringAngle() {
        geometry_msgs::Point vehicle_position = current_position;
        is_look_forward_point = false;
        double local_path_point[3] = {0.0, 0.0, 0.0};
        // rotated_point = geometry_msgs::Point();;
        // Reference of Map and vehicle coordinate conversion
        
        // std::vector<double> translation = {vehicle_position.x, vehicle_position.y};
        // std::vector<std::vector<double>> t = {
        //     {std::cos(vehicle_yaw), -std::sin(vehicle_yaw), translation[0]},
        //     {std::sin(vehicle_yaw), std::cos(vehicle_yaw), translation[1]},
        //     {0, 0, 1}
        // };
        // std::vector<std::vector<double>> det_t = {
        //     {t[0][0], t[1][0], -(t[0][0] * translation[0] + t[1][0] * translation[1])},
        //     {t[0][1], t[1][1], -(t[0][1] * translation[0] + t[1][1] * translation[1])},
        //     {0, 0, 1}
        // };

        std::vector<double> translation = {vehicle_position.x, vehicle_position.y};
        std::vector<std::vector<double>> t = {
            {std::cos(vehicle_yaw), -std::sin(vehicle_yaw), translation[0]},
            {std::sin(vehicle_yaw), std::cos(vehicle_yaw), translation[1]},
            {0, 0, 1}
        };
        std::vector<std::vector<double>> det_t = {
            {t[0][0], t[1][0], -(t[0][0] * translation[0] + t[1][0] * translation[1])},
            {t[0][1], t[1][1], -(t[0][1] * translation[0] + t[1][1] * translation[1])},
            {0, 0, 1}
        };


        for (size_t i = 0; i < path.size(); ++i) {
            const auto& poseStamped = path[i];
            const geometry_msgs::Point& path_point = poseStamped.pose.position;
            std::cout << "pose: " << path_point << std::endl;
            // double dx = path_point.x - vehicle_position.x;
            // double dy = path_point.y - vehicle_position.y;
            // rotated_point.x = std::cos(vehicle_yaw) * dx + std::sin(vehicle_yaw) * dy;
            // rotated_point.y = std::sin(vehicle_yaw) * dx - std::cos(vehicle_yaw) * dy;

            std::vector<double> global_path_point = {path_point.x, path_point.y, 1.0};
            // Convert map reference coordinate data to vehicle reference coordinate data
            
            // std::vector<double> local_path_point(3, 0.0);

            // Convert map reference coordinate data to vehicle reference coordinate data
            local_path_point[0] = det_t[0][0] * global_path_point[0] + det_t[0][1] * global_path_point[1] + det_t[0][2] * global_path_point[2];
            local_path_point[1] = det_t[1][0] * global_path_point[0] + det_t[1][1] * global_path_point[1] + det_t[1][2] * global_path_point[2];
            local_path_point[2] = det_t[2][0] * global_path_point[0] + det_t[2][1] * global_path_point[1] + det_t[2][2] * global_path_point[2];


            if (local_path_point[0] > 0) {
                double dis = std::sqrt(local_path_point[0] * local_path_point[0] + local_path_point[1] * local_path_point[1]);
                // double dis = std::sqrt(rotated_point.x * rotated_point.x + rotated_point.y * rotated_point.y);
                std::cout << "current_vel: " << current_vel << std::endl;
                if (dis >= lfd) {
                    lfd = current_vel * kf;
                    std::cout << "lfd: " << lfd << std::endl;
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
        // double theta = std::atan2(rotated_point.y, rotated_point.x);
        double theta = std::atan2(local_path_point[1], local_path_point[0]);
        std::cout << "theta: " << theta << std::endl;
        if (is_look_forward_point) {
            steering = std::atan2(2 * vehicle_length * std::sin(theta), lfd) * 180 / M_PI;
            std::cout << "Steering_point: " << steering << std::endl;
            return steering;
        } else {
            // ROS_INFO("No forward point found.");
            return 0;
        }
        
    }
private:
    std::vector<geometry_msgs::PoseStamped> path;
    morai_msgs::EgoVehicleStatus statusMsg;

    geometry_msgs::Point forward_point;
    geometry_msgs::Point current_position;  
    // geometry_msgs::Point rotated_point;
    bool is_look_forward_point;
    double vehicle_length;
    double lfd;
    double kf;
    double min_lfd;
    double max_lfd;
    double steering;
    double vehicle_yaw;
    double current_vel;
    // std::vector<double> local_path_point;
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
                // Populate the localPath vector with some data (for demonstration purposes)

                // Print the vector elements
                // for (const auto& pose : localPath) {
                //     std::cout << "Position: (" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << ")" << std::endl;
                // }
                // Assuming the local planner logic will be implemented here.
                // == Tracking control ===========================
                trackingControl.getPath(localPath);
                trackingControl.getEgoStatus(statusMsg);
                ctrlCmdMsg.steering = trackingControl.steeringAngle() / 180.0 * M_PI;
                std::cout << "steering_pub: " << ctrlCmdMsg.steering << std::endl;
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
