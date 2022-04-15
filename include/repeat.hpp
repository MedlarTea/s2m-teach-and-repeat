#pragma once
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <sensor_msgs/Joy.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelStates.h>

/***
 * pd控制器基本思想：当误差较大时，减小线速度，增大角速度；当误差较小时，增大线速度，减小角速度
***/
class pdControl{
public:
    double  x_err, y_err, a_err;  // 偏差值，代表测量量与控制量的误差
    double  x_last_err, y_last_err, a_last_err;  // 上一偏差值
    double x_integral, y_integral, a_integral;  // integral
    double  Kp_v_x, Ki_v_x, Kd_v_x,  Kp_v_y, Ki_v_y, Kd_v_y, Kp_a, Ki_a, Kd_a; // 线速度和角速度-pd控制器的比例, 微分系数, 和积分系数
    double  umax, umin;  // 设置速度的上限与下限
    bool is_recorded;  // 是否为第一次记录
    double x_speed, y_speed, a_speed;  // x, y, a velocity
    geometry_msgs::Twist velocity;  // output velocity



    pdControl();
    pdControl(const std::vector<double> &PD_param_list);
    


    double getYaw(const Eigen::Isometry3d &diff_pose);

    void calError(const Eigen::Isometry3d &diff_pose);

    double getVelocity(double err, double last_err, double integral, double Kp, double Ki, double Kd, double umax, double umin, const std::string &type);

    void vel_cmd(ros::Publisher &pub, ros::Rate &rate);
};


class RepeatNode {
 public:
    std::string posesPath, recordGtPath;  // teach poses path, repeat gt poses path
    double x_threshold,y_threshold,a_threshold;
    bool isStopOutput;  // is outpout information
    bool isStop;  // For save control
    bool isBegin;  // For begining the repeat
    std::list<Eigen::Isometry3d> poses;  // poses to be repeat
    std::size_t poses_length;
    // repeat
    Eigen::Isometry3d current_pose; // current_pose relative to the world
    Eigen::Isometry3d target_pose; // target_pose relative to the world
    Eigen::Isometry3d diff_pose; // current_pose relative to the target_pose
    bool useGazebo;
    std::vector<Eigen::Matrix<double,8,1>> gtPoses;  // record the poses from the gazebo


    RepeatNode();

    RepeatNode(const std::string &posesPath, const std::string &recordGtPath, const std::vector<double> &repeat_param_list, const bool &useGazebo);

    

    void setInitialPose_for_amcl(ros::Publisher &pub, Eigen::Isometry3d initPose);

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    // record properties
    // std::vector<double> pose;  // pose (x, y, yaw) unit(m, m, radius)  radius is (-pi, pi)
    
    // the input is (x,y,z,q_x,q_y,q_z,q_w)
    Eigen::Isometry3d convertToT(std::vector<double> &pose);

    void loadPoses();

    bool isClose(const pdControl& pd);

    void updatePose(const tf::StampedTransform& transform, pdControl& pd);

    void repeat(const tf::StampedTransform& transform, pdControl& p);
    
    //calculate the velocity and angle_velocity
    void calVelocity(pdControl& p);

    // record gt pose from gazebo
    void gazeboPoseCallback();

};
