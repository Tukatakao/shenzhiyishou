#include "topic_ik.hpp"

IkTopicNode::IkTopicNode() : Node("ik_topic_node")
{
    target_sub = this->create_subscription<geometry_msgs::msg::Pointstamped>(
        "/target_position",
        10,
        std::bind(&IkTopicNode::targetcb, this, std::placeholders::_1));

    joint_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        "/ik_joint_command",
        10, );
}
IkTopicNode::targetcb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    std::array<double, 3> jieguo{};
    bool ft = solver.yundongjie(msg->point.x, msg->point.y, msg->point.z, jieguo);

    // 封装发布
    auto joint_msg = sensor_msgs::msg::JointState{};
    joint_msg.header.stamp = this->now();
    joint_msg.header.frame_id = "base_link";
    joint_msg.name = {"b_1", "l1_2", "l2_3"};
    joint_msg.posititon = {jieguo[0], jieguo[1], jieguo[2]};

    joint_pub->publish(joint_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IkTopicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
