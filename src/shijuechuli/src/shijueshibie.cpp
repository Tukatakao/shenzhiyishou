#include "shijuechuli.hpp"

#include <iostream>

shijueshibie::shijueshibie():Node("shijueshibie")
{
    this->declare_parameter("top_plane_z",0.02);
    std::vector<double> waican_r={1,0,0, 0,1,0, 0,0,1};
    std::vector<double> waican_t={0.0, 0.0, 0.5};
    
    this->declare_parameter("xiangji_r",waican_r);
    this->declare_parameter("xiangji_t",waican_t);

    this->declare_parameter("depth_min",0);
    this->declare_parameter("depth_max",0);
 
    cv::Mat frame;

    top_plane_z_=this->get_parameter("top_plane_z").as_double();

    waicanr = this->get_parameter("xiangji_r").as_double_array();
    waicant = this->get_parameter("xiangji_t").as_double_array();

    R1_=cv::Matx33d(waicanr[0],waicanr[1],waicanr[2],waicanr[3],waicanr[4],waicanr[5],waicanr[6],waicanr[7],waicanr[8]);
    t1_=cv::Vec3d(waicant[0],waicant[1],waicant[2]);

    min = this->get_parameter("depth_min").as_double();
    max = this->get_parameter("depth_max").as_double();

   
    cam_info_sub=this->create_subscription<sensor_msgs::msg::CameraInfo>("/top_camera/color/camera_info",10,std::bind(&shijueshibie::neican1,this,std::placeholders::_1));
    depth_sub=this->create_subscription<sensor_msgs::msg::Image>("/top_camera/color/Image_depth_raw",10,std::bind(&shijueshibie::depth_huitiao,this,std::placeholders::_1));

    target_pub=this->create_publisher<geometry_msgs::msg::PointStamped>("/target_weizhi",10);
}


void shijueshibie::neican1(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    K1_=cv::Mat(3,3,CV_64F,(void*)msg->k.data()).clone();
    dist1_=cv::Mat(1,5,CV_64F,(void*)msg->d.data()).clone();
    cam1_ready_=true;
}

void shijueshibie::depth_huitiao(const sensor_msgs::msg::Image::SharedPtr img)


{
    if(!cam1_ready_) return;
    cv::Mat depth1 (img->height,img->width,CV_16U,const_cast<uint8_t*>(img->data.data()),img->step);
    depth=depth1.clone();
}


void shijueshibie::zhaodaozuobiao(cv::Mat depth)
{

    double sx = 0,sy = 0;
    int count = 0;
    for(int v= 0;v < depth.rows;++v)
    {
        for(int u = 0;u < depth.cols;++u)
        {
            uint16_t d = depth.at<uint16_t>(v,u);
            if(d == 0) continue;
            
            if(d >= min && d <= max)
            {
                sx += v;
                sy += u;
                count++;
            }
        }
    }
    zhongxin_x = sx/count;
    zhongxin_y = sy/count;
    
}


void shijueshibie::zuobiaofantouying(double x,double y,cv::Mat K,cv::Mat D)
{
    std::vector<cv::Point2d> zuobiao1;
    zuobiao1.emplace_back(x,y);
    std::vector<cv::Point2d> zuobiao2;
    cv::undistortPoints(zuobiao1,zuobiao2,K1_,dist1_);
    double x_u = zuobiao2[0].x;
    double y_v = zuobiao2[0].y;
    
    cv::Vec3d o(0,0,0);
    cv::Vec3d d(x_u,y_v,1.0);

    
    
    double t = top_plane_z_/1.0;
    cv::Vec3d point(x_u*t,y_v*t,t);
    point_base = R1_*point+t1_;
   
   

}
void shijueshibie::zuobiaofabu(cv::Vec3d point_base)
{
    geometry_msgs::msg::PointStamped target_msg;
    target_msg.header.stamp = this->now();
    target_msg.header.frame_id = "base_link";
    target_msg.point.x = point_base[0];
    target_msg.point.y = point_base[1];
    target_msg.point.z = point_base[2];

    target_pub->publish(target_msg);

}
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<shijueshibie>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows(); // 仅视觉节点需要
    return 0;

}





