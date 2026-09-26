#include "topic_ik.hpp"

IkTopicNode::IkTopicNode() : Node("ik_topic_node")
{
    target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/target_weizhi",
        10,
        std::bind(&IkTopicNode::targetcb, this, std::placeholders::_1));

    joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory",
        10);
}
void IkTopicNode::targetcb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    std::array<double, 3> jieguo{};
    bool ft = solver.yundongjie(msg->point.x, msg->point.y, msg->point.z, jieguo);

    // 封装发布
    auto traj = trajectory_msgs::msg::JointTrajectory{};
    traj.header.stamp = this->now();
    traj.joint_names = {"b_1", "l1_2", "l2_3"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {jieguo[0], jieguo[1], jieguo[2]};
    point.positions = {jieguo[0], jieguo[1], jieguo[2]};
    if (ft)
    {
        traj.points.push_back(point);
        joint_pub_->publish(traj);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IkTopicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}