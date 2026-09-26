#ifndef SHI_JUE_CHU_LI
#define SHI_JUE_CHU_LI

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>







class shijueshibie:public rclcpp::Node
{
   public:
   shijueshibie();
   private:
   void depth_huitiao(const sensor_msgs::msg::Image::SharedPtr img);       //深度回调
   
   void neican1(const sensor_msgs::msg::CameraInfo::SharedPtr msg);        //内参回调


   void zuobiaofantouying(double x,double y,cv::Mat K,cv::Mat D);
   void zhaodaozuobiao(cv::Mat depth);          //坐标回调
   void zuobiaofabu(cv::Vec3d point_base);


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
   
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;
   
    
    cv::Mat K1_, dist1_;
    cv::Matx33d R1_;  // 相机→基座 旋转
    cv::Vec3d t1_;    // 相机→基座 平移
    bool cam1_ready_ = false;
    std::vector<double> waicanr;
    std::vector<double> waicant;
   

    double top_plane_z_;   

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub; 

    double min;
    double max;
    double zhongxin_x;
    double zhongxin_y;

    cv::Mat depth;
    cv::Vec3d point_base;

};
#endif