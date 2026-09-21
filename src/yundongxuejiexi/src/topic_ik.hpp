#ifndef TOPIC_OUTPUT
#define TOPIC_OUTPUT

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/potint_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ik.hpp"

class IkTopicNode : public rclcpp::Node
{
public:
IkTopicNode() private : void targetcb(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    Kinematics solver;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
}

#endif