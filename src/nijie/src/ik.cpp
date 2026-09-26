// 三轴机器人运动学反解
#include "ik.hpp"
#include <iostream>


Kinematics::Kinematics() : dabi{0.3}, xiaobi{0.2}, link2_begin{0.315}
{
    link1_limit = {0.0, 1.5708};
    link2_limit = {-0.5236, 1.5708};
    link3_limit = {-1.5708, 1.5708};
}
bool Kinematics::yundongjie(double x, double y, double z, std::array<double, 3> &joint)
{
        //计算腰关节
        double theta1 = atan2(y, x); // 计算xy平面以决定腰部旋转角度
        if (theta1 < link1_limit[0] - eps || theta1 > link1_limit[1] + eps)
        {
            std::cerr << "jointb_1超出旋转上限" << std::endl;
            return false;
        }
        // 投影到竖直平面
        double r = sqrt(x * x + y * y);
        double h = z - link2_begin;
        double c = sqrt(r * r + h * h); // 竖直平面上点到link2起始点的距离

        // 根据上述所计算距离计算link2和link3长度是否足够
        double min_reach = fabs(dabi - xiaobi);
        double max_reach = fabs(dabi + xiaobi);
        if (c < min_reach - eps || c > max_reach + eps)
        {
            std::cerr << "该物体超出抓取范围" << std::endl;
            return false;
        }
        // 求解肘关节,用三角形三边定理。 大臂长，小臂长，以及物体到大臂的距离三边确定，求的是外角
        double cos_theta3 = (c *c - dabi *dabi - xiaobi *xiaobi ) / (2 *dabi *xiaobi);
        // 夹值
        cos_theta3 = std::max(-1.0 + eps, std::min(1.0 - eps, cos_theta3)); // 嵌套夹值，内层如果取小 截断超过最大值的情况，//外层取大，截断小于最小值的情况
        double theta3 = acos(cos_theta3);    //虽然有多种解，但是只取负，因为很多时候机械只允许一种
        //theta3限位检查
        if(theta3 < link3_limit[0] - eps || theta3 > link3_limit[1] + eps)
        {
            std::cerr << "joint2_3超出旋转上限" << std::endl;
            return false;           
        }
        //求解肩关节,也是三角形
        double tan_theta2 = atan2(h,r);
        double cos_theta2 = (c*c + dabi*dabi - xiaobi*xiaobi)/(2*dabi*c);
        cos_theta2 = std::max (-1.0+eps,std::min(1.0-eps,cos_theta2));
        double theta2_cos = acos(cos_theta2);
        double pi=3.14159265358979323846;
        double theta2 = pi/2 - theta2_cos - tan_theta2;
        
        if(theta2 <link2_limit[0]-eps || theta2 > link2_limit[1]+eps)
        {
            std::cerr << "joint2_3超出旋转上限" << std::endl;
            return false;
        }
        joint[0] = theta1;
        joint[1] = theta2;
        joint[2] = theta3;
        return true;   
}   
    

std::array<double,3> Kinematics::forward(const std::array<double, 3> &joint_angles)
{
        
        double yao = joint_angles[0];
        double jian= joint_angles[1];
        double zhou= joint_angles[2];

        double link3_base_r = dabi* sin(jian);
        double link3_base_h = dabi * cos(jian);
        
        
        double link3_tip_x = xiaobi *  sin(jian+zhou); 
        double link3_tip_y = xiaobi * -cos(jian+zhou);


        double tip_link2_begin_r =  link3_base_r + link3_tip_x;
        double tip_link2_begin_h =  link3_base_h + link3_tip_y;
        

        double x =  tip_link2_begin_r * cos(yao); 
        double y =  tip_link2_begin_r * sin(yao);        
        double z =  tip_link2_begin_h + link2_begin;

        return {x,y,z};

}



    


 

