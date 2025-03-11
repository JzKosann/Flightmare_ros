#pragma once

#include <ros/ros.h>
#include <thread>
#include <mav_msgs/QuadCurState.h>

namespace acados_ros{
    void UnityCallback(const mav_msgs::QuadThrusts::ConstPtr &msg);
    // 订阅
    ros::Subscriber QuadThrusts_sub;
    // 发布
    ros::Publisher QuadCurStates_pub;
    
    void quaternionToEuler(double q_w, double q_x, double q_y, double q_z, double &roll, double &pitch, double &yaw) {
        // 计算 Roll (X 轴旋转)
        double sinr_cosp = 2 * (q_w * q_x + q_y * q_z);
        double cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y);
        roll = std::atan2(sinr_cosp, cosr_cosp);
    
        // 计算 Pitch (Y 轴旋转)
        double sinp = 2 * (q_w * q_y - q_z * q_x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // 防止超出范围
        else
            pitch = std::asin(sinp);
    
        // 计算 Yaw (Z 轴旋转)
        double siny_cosp = 2 * (q_w * q_z + q_x * q_y);
        double cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    // void render_process();

    // std::thread render_thread{render_process};
    
}

