#ifndef SHI_JUE_CHI_LI
#define SHI_JUE_CHU_LI

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>





class shijueshibie::public rclcpp::Node
{
   public:
   Shijueshibie();
   private:
   void tuxiangzhuanhuan(const sensor_msgs::msg::Image::SharedPtr img_top,
   const sensor_msgs::msg::Image::SharedPtr img_side);
   void neican1(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
   //void neican2(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

   bool zuobiaofantouying(int u,int v,const cv::Mat& k,const cv::Mat &dist,const cv::Matx33d& R, const cv::Vec3d& t,const cv::Vec3d& plane_normal, double plane_d,cv::Vec3d& point_base);
   bool zhaodaozuobiao(const cv::Mat& img, cv::Point& center_px);


    //成员变量？
    //message_filters::Subscriber<sensor_msgs::msg::Image> top_sub_;    //filter的收发
    
    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    //std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub_;
   
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
   
    
    cv::Mat K1_, dist1_;
    cv::Matx33d R1_;  // 相机→基座 旋转
    cv::Vec3d t1_;    // 相机→基座 平移
    bool cam1_ready_ = false;

   

    double top_plane_z_;   

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_; 

    int h_min1_, h_max1_, h_min2_, h_max2_;
    int s_min_, v_min_;
    int min_area_;

    cv::Mat frame;







};
#endif