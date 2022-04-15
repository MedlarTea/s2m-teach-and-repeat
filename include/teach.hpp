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
/**
*@brief record the map and poses
*/
class Teacher {
 public:
    std::string recordPath, recordGtPath, recordMapPath; // the path to store the teach_poses, teach_gt_poses, and map file
    double disThreshold, yawThreshold;  // threshold to record the poses
    bool useGazebo;  // useGazebo, then we will record the gt poses in recordGtPath
    bool isRecording;  // recording state
    bool beginRecord;  // used to control whether start record the trajectory
    // Eigen::Matrix<double,7,1> pose;  // pose (x, y, yaw) unit(m, m, radius)  radius is (-pi, pi)
    std::vector<Eigen::Matrix<double,7,1>> poses;  // poses to be repeat
    std::vector<Eigen::Matrix<double,8,1>> gtPoses;



    Teacher();

    Teacher(const std::string &recordPath, const std::string &recordGtPath, const std::string &recordMapPath, const double &disThreshold, const double &yawThreshold, const bool &useGazebo);

    

    // change the tf::StampedTransform to (x,y,z, q_x, q_y, q_z, q_w)
    void preprocessPose(const tf::StampedTransform& tf);

    // record the teaching poses
    void recordPoses();

    //subscribe the ps4's buttons, and change the state if triggered
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

    // record gt pose from gazebo
    void gazeboPoseCallback();
    
    // save the map and exit the node
    void recordMap();

};
