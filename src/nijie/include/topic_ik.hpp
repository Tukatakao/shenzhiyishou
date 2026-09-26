#ifndef TOPIC_OUTPUT
#define TOPIC_OUTPUT

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>  

// 成员类型改
rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
#include "ik.hpp"

class IkTopicNode:public rclcpp::Node
{
public:
IkTopicNode(); 
private:
void targetcb(const geometry_msgs::msg::PointStamped::SharedPtr msg);
Kinematics solver;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;

rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
};

#endif