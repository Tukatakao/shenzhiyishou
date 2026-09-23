#include "shijuechuli.hpp"

#include <iostream>

shijueshibie::shijueshibie():Node("shijueshibie")
{
    this->declare_parameter("top_plane_z",0.02);
    std::vector<double> waican_r={1,0,0, 0,1,0, 0,0,1};
    std::vector<double> waican_t={0.0, 0.0, 0.5};
    
    this->declare_parameter("xiangji_r",waican_r);
    this->declare_parameter("xiangji_t",waican_t);

    this->declare_parameter("h_min1",0);
    this->declare_parameter("h_max1",0);
    this->declare_parameter("h_min2",0);
    this->declare_parameter("h_max2",0);
    this->declare_parameter("s_min",0);
    this->declare_parameter("v_min",0);
    this->declare_parameter("min_area",0);

    top_plane_z_=this->get_parameter("top_plane_z").as_double();

    auto waicanr = this->get_parameter("xiangji_r").as_double_array();
    auto waicant = this->get_parameter("xiangji_t").as_double_array();

    R1_=cv::Matx33d(waicanr[0],waicanr[1],waicanr[2],waicanr[3],waicanr[4],waicanr[5],waicanr[6],waicanr[7],waicanr[8]);
    t1_=cv::Vec3d(waicant[0],waicant[1],waicant[2]);

    h_min1_ = this->get_parameter("h_min1").as_int();
    h_max1_ = this->get_parameter("h_max1").as_int();
    h_min2_ = this->get_parameter("h_min2").as_int();
    h_max2_ = this->get_parameter("h_max2").as_int();
    s_min_ = this->get_parameter("s_min").as_int();
    v_min_ = this->get_parameter("v_min").as_int();
    min_area_ = this->get_parameter("min_area").as_int();
    /*
    top_sub_.subsribe(this,"/top_camera/color/image_raw");
    sync_=std 
    */
    cam_info_sub=this->create_subscription<sensor_msgs::msg::CameraInfo>("/top_camera/color/camera_info",10,std::bind(&shijueshibie::neican1,this,std::placeholders::_1));
Image_sub_=this->create_subscription<sensor_msgs::msg::Image>("/top_camera/color/Image_raw",10,std::bind(&shijueshibie::tuxiangzhuanhuan,this,std::placeholders::_1));

    target_pub=this->create_publisher<geometry_msgs::msg::PointStamped>("/target_weizhi",10);
}


void shijueshibie::neican1(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    K1_=cv::Mat(3,3,CV_64F,(void*)msg->k.data()).clone();
    dist1_=cv::Mat(1,5,CV_64F,(void*)msg->d.data()).clone();
    cam1_ready_=true;
    cam_info_sub_.reset();
}
void shijueshibie::tuxiangzhuanhuan(const sensor_msgs::msg::Image::SharedPtr img)

{
    if(!cam1_ready_) return;
    
    try{
        frame = cv_bridge::toCvCopy(img,"bgr8")->image;

    }
    catch(cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "图像转换失败: %s", e.what());
        return;
    } 
}






