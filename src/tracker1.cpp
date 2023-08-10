#include <iostream>
#include "ros/ros.h"
#include <cmath>

#include "./localization/point.h"
#include "./localization/vehicle_state.h"
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
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>

#include "std_msgs/String.h"
#include <sstream>

void vehicle_Status_Callback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
  VehicleState vehicle_state(msg->position.x, msg->position.y, msg->heading*M_PI/180.0, msg->velocity.x);
  std::cout << vehicle_state.str() << std::endl;

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "gps";
  transformStamped.transform.translation.x = vehicle_state.position().x();
  transformStamped.transform.translation.y = vehicle_state.position().y();
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, vehicle_state.yaw());
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  br.sendTransform(transformStamped);
  
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "tracker");

  ros::NodeHandle n, m;
// =============== Subscriber =============================
  ros::Subscriber ego_status_sub = n.subscribe<morai_msgs::EgoVehicleStatus>("/Ego_topic", 10, vehicle_Status_Callback);


// =============== Publisher =============================
  ros::Publisher ctrl_cmd_pub = m.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 10);

  ros::Rate loop_rate(10); // Set the publishing rate (10 Hz)

  int count = 0;
  while (ros::ok())
  {

    morai_msgs::CtrlCmd controlCmd;

    // Set the fields of the message as needed
    controlCmd.steering = 0.2;
    controlCmd.accel = 1.0;
    controlCmd.brake = 0.0;
    std::cout << controlCmd << std::endl;

    // Publish the message
    ctrl_cmd_pub.publish(controlCmd);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  ros::spin();

  return 0;
}
