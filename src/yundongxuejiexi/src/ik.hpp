#ifndef KINEMATICS_SOLVER_H
#define KINEMATICS_SOLVER_H

#include <array>
#include <cmath>

class Kinematics
{

public:
    Kinematics();
    bool yundongjie(double x, double y, double z, std::array<double, 3> &joint_angles); // 逆向求解
    std::array<double, 3> forward(const std::array<double, 3> &joint_angles);           // 正向求解

private:         // 机械臂的dh参数是私人的 不能随便改
    double dabi; // 不写死，不同长度机械臂也能使用
    double xiaobi;
    double link2_begin;

    std::array<double, 2> link1_limit;
    std::array<double, 2> link2_limit;
    std::array<double, 2> link3_limit;
    static constexpr double eps =1e-6; //保证反算角度正常
};

#endif
