#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <string>

class PathReader {
public:
    PathReader(const std::string& file_path) : file_path_(file_path) {}

    nav_msgs::Path readTxt(const std::string& file_path) {
        std::string full_file_name = "/home/rml-phat/catkin_ws/src/MORAI/erp_cpp/path/lidar_test.txt";
        
        std::ifstream openFile(full_file_name.c_str());
        nav_msgs::Path out_path;
        out_path.header.frame_id = "/map";
        std::string line;

        while (getline(openFile, line)) {
            std::istringstream iss(line);
            geometry_msgs::PoseStamped read_pose;
            double x, y, z;
            if (iss >> x >> y >> z) {
                read_pose.pose.position.x = x;
                read_pose.pose.position.y = y;
                read_pose.pose.position.z = z;
                read_pose.pose.orientation.x = 0;
                read_pose.pose.orientation.y = 0;
                read_pose.pose.orientation.z = 0;
                read_pose.pose.orientation.w = 1;
                out_path.poses.push_back(read_pose);
            }
        }

        openFile.close();
        return out_path;
    }

private:
    std::string file_path_;
};
