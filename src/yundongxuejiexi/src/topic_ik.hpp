#ifndef TOPIC_OUTPUT
#define TOPIC_OUTPUT

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/potint_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ik.hpp"

class IkTopicNode : public rclcpp::Node
{
public:
IkTopicNode() 
private : 
void targetcb(const geometry_msgs::msg::PointStamped::SharedPtr msg);
Kinematics solver;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

cv::Mat K1_, dist1_;
cv::Matx33d R1_;
cv::Vec3d t1_;
bool cam1_ready_ = false;

cv::Mat K2_, dist2_;
cv::Matx33d R2_;
cv::Vec3d t2_;
bool cam2_ready_ = false;

double top_plane_z_;
double side_plane_x_;


rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;


int h_min1_, h_max1_, h_min2_, h_max2_;
int s_min_, v_min_;
int min_area_;


}

#endif