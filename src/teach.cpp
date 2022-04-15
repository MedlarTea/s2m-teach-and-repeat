#include <fstream>
#include <iomanip>
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <unistd.h>

#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>

#include "teach.hpp"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>

Teacher::Teacher(const std::string &recordPath, const std::string &recordGtPath, const std::string &recordMapPath, const double &disThreshold, const double &yawThreshold, const bool &useGazebo)
{
  this->recordPath = recordPath;
  this->recordGtPath = recordGtPath;
  this->recordMapPath = recordMapPath;
  this->disThreshold = disThreshold;
  this->yawThreshold = yawThreshold;
  this->useGazebo = useGazebo;
}

void Teacher::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  std::vector<int> buttons = msg->buttons; // (cha, circle, triangle, square)--(buttons 0,1,2,3)
  if (buttons[0]==1 && !isRecording){
    isRecording = true;
    recordPoses();
    std::cout << "Recorded the poses" << std::endl;
    recordMap();
    std::cout << "Recorded the Map" << std::endl;
  }
  if (buttons[1]==1 && !isRecording){
    std::cout << "Begin the teach" << std::endl;
    beginRecord = true;
  }
}

void Teacher::gazeboPoseCallback()
{
  // service model,适用于这种瞬间触发的信息获取
  ros::NodeHandle n2;
  ros::ServiceClient client2 = n2.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gazebo_msgs::GetModelState srv2;
  srv2.request.model_name = "dingo"; //指定要获取的机器人在gazebo中的名字；

  if (client2.call(srv2))
    {
      gazebo_msgs::GetModelStateResponse state = srv2.response;
      geometry_msgs::Pose gtPose = state.pose;
      geometry_msgs::Point gtPose_t = gtPose.position;
      geometry_msgs::Quaternion gtPose_q = gtPose.orientation;
      double timestamp = state.header.stamp.toSec();
      Eigen::Matrix<double,8,1> p;
      p << timestamp, gtPose_t.x, gtPose_t.y, gtPose_t.z, gtPose_q.x, gtPose_q.y, gtPose_q.z, gtPose_q.w;
      gtPoses.push_back(p);
    }
  else
  {
    ROS_ERROR("Failed to call service /gazebo/get_model_state");
  }
}

void Teacher::preprocessPose(const tf::StampedTransform& transform)
{
  if (!beginRecord)
    return;
  double x = transform.getOrigin().x();
  double y = transform.getOrigin().y();
  double z = transform.getOrigin().z();
  

  double q_x = transform.getRotation().getX();
  double q_y = transform.getRotation().getY();
  double q_z = transform.getRotation().getZ();
  double q_w = transform.getRotation().getW();
  Eigen::Matrix<double,7,1> pose;
  pose << x,y,z,q_x,q_y,q_z,q_w;
  // double roll, pitch, yaw;
  // tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
  if (poses.empty())
  {
    poses.push_back(pose);
    if (useGazebo)
      gazeboPoseCallback();  // get the gt poses from gazebo
    std::cout << std::setprecision(4) << x << " "
              << std::setprecision(4) << y << " "
              << std::setprecision(4)<< q_z << std::endl;
  }
  else
  {
    double last_x = poses.back()(0,0);
    double last_y = poses.back()(1,0);
    // double last_yaw = poses.back()(2);
    double x_dis = abs(x - last_x);
    double y_dis = abs(y - last_y);
    // double dif_yaw =  abs((yaw - last_yaw));
    if (x_dis >= disThreshold || y_dis >= disThreshold)
    {
      poses.push_back(pose);
      if (useGazebo)
        gazeboPoseCallback();  // get the gt poses from gazebo
      // std::cout << std::setprecision(4) << std::to_string(dis) << " " << std::to_string(dif_yaw) << std::endl;
      std::cout << std::setprecision(4) << x << " "
              << std::setprecision(4) << y << " "
              << std::setprecision(4)<< q_z << std::endl;
    }
  }
}

void Teacher::recordPoses()
{
  std::ofstream f_poses, f_gtposes;
  f_poses.open(recordPath.c_str());
  if (useGazebo)
    f_gtposes.open(recordGtPath.c_str());
  for(size_t i=0;i<poses.size();i++)
  {
    f_poses << std::setprecision(7) << poses[i](0,0) << " " << poses[i](1,0) << " " << poses[i](2,0) << " " << poses[i](3,0) << " " << poses[i](4,0) << " " << poses[i](5,0) << " " << poses[i](6,0) << "\n";
    if (useGazebo)
      f_gtposes << std::setprecision(6) << gtPoses[i](0,0) << std::setprecision(7) << " " << gtPoses[i](1,0) << " " << gtPoses[i](2,0) << " " << gtPoses[i](3,0) << " " << gtPoses[i](4,0) << " " << gtPoses[i](5,0) << " " << gtPoses[i](6,0) << " " << gtPoses[i](7,0) << "\n";
  }
  f_poses.close();
  if (useGazebo)
    f_gtposes.close();
}

void Teacher::recordMap()
{
  std::cout << "Map is in: " << recordMapPath.c_str() << std::endl;
  execlp("rosrun", "rosrun", "map_server", "map_saver", "-f", recordMapPath.c_str(), NULL);
}
