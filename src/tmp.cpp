#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <fstream>

class PathReader {
public:
    PathReader(const std::string& pkg_name) {
        // Get the current file path
        std::string full_file_path = __FILE__;
        size_t pos = full_file_path.find("src");
        if (pos != std::string::npos) {
            pkg_path_ = full_file_path.substr(0, pos);
        }
        pkg_path_ += "path/" + pkg_name;
    }

    nav_msgs::Path read_txt(const std::string& file_name) {
        std::string full_file_name = pkg_path_ + "/" + file_name;
        std::ifstream openFile(full_file_name.c_str());
        nav_msgs::Path out_path;
        out_path.header.frame_id = "map";

        if (openFile.is_open()) {
            std::string line;
            while (std::getline(openFile, line)) {
                std::istringstream iss(line);
                geometry_msgs::PoseStamped read_pose;
                iss >> read_pose.pose.position.x >> read_pose.pose.position.y >> read_pose.pose.position.z;
                read_pose.pose.orientation.x = 0;
                read_pose.pose.orientation.y = 0;
                read_pose.pose.orientation.z = 0;
                read_pose.pose.orientation.w = 1;
                out_path.poses.push_back(read_pose);
            }
            openFile.close();
        }

        return out_path;
    }

private:
    std::string pkg_path_;
};

class PurePursuit {
public:
    PurePursuit() {
        // Initialize other variables...
    }

    void getPath(const nav_msgs::Path& msg) {
        path_ = msg;
    }

    void getEgoStatus(const morai_msgs::EgoVehicleStatus& msg) {
        // Update ego status variables...
    }

    double steering_angle() {
        // Calculate steering angle using purePursuit algorithm...
        // Return the calculated steering angle
    }

private:
    // Define other variables and functions...
};

class ErpPlanner {
public:
    ErpPlanner() {
        ros::NodeHandle nh("~");

        std::vector<std::string> arg;
        nh.getParam("argv", arg);

        if (arg.size() < 3) {
            ROS_ERROR("Not enough command line arguments!");
            return;
        }

        path_name_ = arg[1];
        traffic_control_ = arg[2];

        // Publisher
        global_path_pub_ = nh.advertise<nav_msgs::Path>("/global_path", 1);
        local_path_pub_ = nh.advertise<nav_msgs::Path>("/local_path", 1);
        ctrl_pub_ = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);

        // Subscriber
        ego_status_sub_ = nh.subscribe("/Ego_topic", 1, &ErpPlanner::statusCB, this);

        // Initialize variables
        is_status_ = true;
        is_traffic_ = false;

        // Create instances of other classes
        path_reader_ = PathReader("erp_py");
        global_path_ = path_reader_.read_txt(path_name_ + ".txt");
        pure_pursuit_ = PurePursuit();
        status_msg_ = morai_msgs::EgoVehicleStatus();

        count_ = 0;
        rate_ = ros::Rate(10); // 30hz
    }

    void run() {
        while (ros::ok()) {
            if (is_status_) {
                nav_msgs::Path local_path;
                int current_waypoint;
                std::tie(local_path, current_waypoint) = findLocalPath(global_path_, status_msg_);

                pure_pursuit_.getPath(local_path);
                pure_pursuit_.getEgoStatus(status_msg_);

                morai_msgs::CtrlCmd ctrl_msg;
                ctrl_msg.steering = -pure_pursuit_.steering_angle() / 180.0 * M_PI;

                double control_input = 1.5;

                if (control_input > 0) {
                    ctrl_msg.accel = control_input;
                    ctrl_msg.brake = 0;
                } else {
                    ctrl_msg.accel = 0;
                    ctrl_msg.brake = -control_input;
                }

                local_path_pub_.publish(local_path);
                ctrl_pub_.publish(ctrl_msg);

                steering_angle_ = ctrl_msg.steering;
            }

            global_path_pub_.publish(global_path_);

            count_++;
            rate_.sleep();
        }
    }

private:
    ros::Publisher global_path_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher ctrl_pub_;
    ros::Subscriber ego_status_sub_;
    nav_msgs::Path global_path_;
    PathReader path_reader_;
    PurePursuit pure_pursuit_;
    morai_msgs::EgoVehicleStatus status_msg_;
    bool is_status_;
    bool is_traffic_;
    std::string path_name_;
    std::string traffic_control_;
    int count_;
    ros::Rate rate_;

    void statusCB(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
        status_msg_ = *msg;
        is_status_ = true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "erp42_total");
    ErpPlanner erp_planner;
    erp_planner.run();
    return 0;
}
