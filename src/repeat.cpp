#include <fstream>
#include <iomanip>
#include <math.h>
#include <algorithm>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <unistd.h>

#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>

#include "repeat.hpp"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

#define PI 3.1415926

RepeatNode::RepeatNode(const std::string &posesPath, const std::string &recordGtPath, const std::vector<double> &repeat_param_list, const bool &useGazebo)
{   
    this->posesPath = posesPath;
    this->recordGtPath = recordGtPath;
    this->x_threshold = repeat_param_list[0];
    this->y_threshold = repeat_param_list[1];
    this->a_threshold = repeat_param_list[2];
    this->useGazebo = useGazebo;
    loadPoses();
}

// topic
void RepeatNode::setInitialPose_for_amcl(ros::Publisher &pub, Eigen::Isometry3d initPose)
{
    // give the initial pose to the amcl
    geometry_msgs::PoseWithCovarianceStamped poseInitial;  
    geometry_msgs::Pose pose_geometry;
    tf::poseEigenToMsg(initPose, pose_geometry);
    poseInitial.header.frame_id = "map";
    poseInitial.pose.pose = pose_geometry;
    pub.publish(poseInitial);
}

void RepeatNode::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  std::vector<int> buttons = msg->buttons; // (cha, circle, triangle, square)--(buttons 0,1,2,3)
  if (buttons[0]==1){
    isStop = true;
    std::cout << "STOP!!!" << std::endl;
    isStopOutput = true;
  }
  else if (buttons[1]==1){
    isStop = false;
    std::cout << "Restore the repeat" << std::endl;
    isStopOutput = false;
  }
  else if (buttons[2]==1){
    isBegin = true;
    isStop = false;
    std::cout << "Begin the repeat" << std::endl;
  }
}

Eigen::Isometry3d RepeatNode::convertToT(std::vector<double> &pose)
{   
    Eigen::Vector3d t(pose[0],pose[1],pose[2]);
    Eigen::Quaterniond q(pose[6],pose[3],pose[4],pose[5]);
    Eigen::Isometry3d matrix = Eigen::Isometry3d::Identity();
    matrix.rotate(q);
    matrix.pretranslate(t);
    return matrix;
}

void RepeatNode::loadPoses()
{
    std::fstream f;
    std::string line;
    std::string delimiter = " ";
    f.open(posesPath.c_str());
    std::cout << "Loading poses" << std::endl;
    while(getline(f, line))
    {   
        // std::cout << line << std::endl;
        size_t pos = 0;
        std::string token;
        std::vector<double> pose;
        while ((pos = line.find(delimiter)) != std::string::npos) {
            token = line.substr(0, pos);
            // std::cout << token << std::endl;
            pose.push_back(std::stod(token));  // x y
            line.erase(0, pos + delimiter.length());
        }
        pose.push_back(std::stod(line));
        poses.push_back(convertToT(pose));
        // std::cout << line << std::endl;
    }
    poses_length = poses.size();
    std::cout << "Loaded " << poses_length << " poses" << std::endl;
}

bool RepeatNode::isClose(const pdControl& pd)
{
    // if (abs(currentP[0]-targetP[0])<0.0001 && abs(currentP[1]-targetP[1])<0.0001 && abs(currentP[2]-targetP[2])<0.001)
    if (abs(pd.x_err)<x_threshold && abs(pd.y_err)<y_threshold && abs(pd.a_err)<a_threshold)
        return true;
    return false;
}

void RepeatNode::gazeboPoseCallback()
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

void RepeatNode::updatePose(const tf::StampedTransform& transform, pdControl& pd)
{   
    tf::transformTFToEigen(transform, current_pose);
    Eigen::Isometry3d currentInv = current_pose.inverse();
    // diff_pose = target_pose*currentInv;
    diff_pose = currentInv*target_pose;

    // double yaw, pitch, roll;
    // tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    // current_pose.clear();
    // current_pose.push_back(x);
    // current_pose.push_back(y);
    // current_pose.push_back(yaw);
    if (poses.empty() && !isStopOutput)
    {
        std::cout << "Complete Repeat" << std::endl;
        isStopOutput = true;
        isStop = true;
        if (useGazebo)
        {
            std::ofstream f_gtposes;
            f_gtposes.open(recordGtPath.c_str());
            for(size_t i=0;i<gtPoses.size();i++)
            {
                f_gtposes << std::setprecision(6) << gtPoses[i](0,0) << std::setprecision(7) << " " << gtPoses[i](1,0) << " " << gtPoses[i](2,0) << " " << gtPoses[i](3,0) << " " << gtPoses[i](4,0) << " " << gtPoses[i](5,0) << " " << gtPoses[i](6,0) << " " << gtPoses[i](7,0) << "\n";
            }
            f_gtposes.close();
        }
    }
    if(poses.size()==poses_length || isClose(pd) && !poses.empty())
    {   
        target_pose = poses.front();
        // record the gt pose
        if (useGazebo)
            gazeboPoseCallback();
        poses.pop_front();
        diff_pose = currentInv*target_pose;
        // when update waypoint, clear the integral
        pd.x_integral = 0;
        pd.y_integral = 0;
        pd.a_integral = 0;
    }
    if (!isStopOutput)
    {
        std::cout << "current: " << current_pose(0,3) << " " << current_pose(1,3) << " " << pd.getYaw(current_pose)/PI*180 << std::endl;
        std::cout << "target: " << target_pose(0,3) << " " << target_pose(1,3) << " " << pd.getYaw(target_pose)/PI*180 << std::endl;
    }
}

void RepeatNode::repeat(const tf::StampedTransform& transform, pdControl& pd)
{   
    updatePose(transform, pd);
    pd.calError(diff_pose);
    // spin to the teaget direction, then go forward
    if(abs(pd.a_err)<0.1 && !isStop)
    {
        pd.x_speed = pd.getVelocity(pd.x_err, pd.x_last_err, pd.x_integral, pd.Kp_v_x, pd.Ki_v_x, pd.Kd_v_x, pd.umax, pd.umin, "pi");
        pd.y_speed = pd.getVelocity(pd.y_err, pd.y_last_err, pd.y_integral, pd.Kp_v_y, pd.Ki_v_y, pd.Kd_v_y, pd.umax, pd.umin, "pi");
        pd.a_speed = pd.getVelocity(pd.a_err, pd.a_last_err, pd.a_integral, pd.Kp_a, pd.Ki_a, pd.Kd_a, 1.0, 0, "pd");
    }
    else if(abs(pd.a_err)>=0.1 && !isStop)
    {
        pd.x_speed = 0;
        pd.y_speed = 0;
        pd.x_integral = 0;
        pd.y_integral = 0;
        pd.a_speed = pd.getVelocity(pd.a_err, pd.a_last_err, pd.a_integral, pd.Kp_a, pd.Ki_a, pd.Kd_a, 1.0, 0, "pd");

    }
    else
    {
        pd.x_speed = 0;
        pd.y_speed = 0;
        pd.a_speed = 0;
    }
    
    if (!isStopOutput)
    {
        std::cout << "x_speed: " << pd.x_speed << std::endl;
        std::cout << "y_speed: " << pd.y_speed << std::endl;
        std::cout << "a_speed: " << pd.a_speed << std::endl;
        // std::cout << "pd.a_err: " << pd.a_err << "pd.a_last_err: " << pd.a_last_err << std::endl;
    }

}
 


pdControl::pdControl(const std::vector<double> &PD_param_list)
{
    this->Kp_v_x = PD_param_list[0];
    this->Ki_v_x = PD_param_list[1];
    this->Kd_v_x = PD_param_list[2];

    this->Kp_v_y = PD_param_list[3];
    this->Ki_v_y = PD_param_list[4];
    this->Kd_v_y = PD_param_list[5];

    this->Kp_a = PD_param_list[6];
    this->Ki_a = PD_param_list[7];
    this->Kd_a = PD_param_list[8];

    this->umax = PD_param_list[9];
    this->umin = PD_param_list[10];
}

double pdControl::getYaw(const Eigen::Isometry3d &pose)
{
    Eigen::Matrix3d rot = pose.linear();
    tf::Matrix3x3 rot_tf;
    tf::matrixEigenToTF(rot, rot_tf);
    double roll,pitch,yaw;
    rot_tf.getRPY(roll,pitch,yaw);
    return yaw;
}

void pdControl::calError(const Eigen::Isometry3d &diff_pose)
{
    double x_err_tmp = diff_pose(0,3);
    double y_err_tmp = diff_pose(1,3);
    double a_err_tmp = getYaw(diff_pose);
    // integral
    x_integral += x_err_tmp;
    y_integral += y_err_tmp;
    a_integral += a_err_tmp;
    // std::cout << "diff: " << x_err_tmp << " " << y_err_tmp << " " << a_err_tmp/PI*180 << std::endl;
    if (!is_recorded)
    {   
        x_err = x_err_tmp;
        y_err = y_err_tmp;
        a_err = a_err_tmp;
        x_last_err = x_err;
        y_last_err = y_err;
        a_last_err = a_err;
        is_recorded = true;
    }
    else
    {
        x_last_err = x_err;
        y_last_err = y_err;
        a_last_err = a_err;
        x_err = x_err_tmp;
        y_err = y_err_tmp;
        a_err = a_err_tmp;
    }
}

double pdControl::getVelocity(double err, double last_err, double integral, double Kp, double Ki, double Kd, double umax, double umin, const std::string &type)
{
    double speed;
    if (type=="pd")
        speed=Kp*err+Kd*(err-last_err);  // pd-control
    else if (type=="pi")
        speed=Kp*err+Ki*integral;  // pi-control
  
    if (isnan(speed))
        speed=umax;
    else if (abs(speed)<umin)
    {
        if (speed<0)
            speed=-umin;
        else
            speed = umin;
    }
    else if (abs(speed)>umax)
    {
        if(speed<0)
            speed=-umax;
        else
            speed=umax;
    }
    return speed;
}

void pdControl::vel_cmd(ros::Publisher &pub, ros::Rate &rate)
{
    velocity.linear.x = x_speed;
    velocity.linear.y = y_speed;
    velocity.angular.z = a_speed;
    pub.publish(velocity);
    rate.sleep();
}
