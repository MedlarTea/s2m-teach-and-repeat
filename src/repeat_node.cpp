#include <ros/ros.h>
#include "repeat.hpp"

#include <sensor_msgs/Joy.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc,char** argv)
{   
    // node initialization
    ros::init(argc,argv,"repeat");
    ros::NodeHandle node;

    // get the param from the launch file
    // for PD control
    std::vector<double> PD_param_list;  // [Kp_v_x, Ki_v_x, Kd_v_x, Kp_v_y, Ki_v_y, Kd_v_y, Kp_a, Ki_a, Kd_a, umax, umin]
    node.getParam("PD_param_list", PD_param_list);
    for (size_t i=0;i<PD_param_list.size();i++)
        std::cout << PD_param_list[i] << std::endl;
    // for repeat node initialization
    std::string posesPath, recordGtPath;
    std::vector<double> repeat_param_list;  // [x_threshold, y_threshold, a_threshold]
    bool useGazebo; // record poses from gazebo?
    node.getParam("posesPath", posesPath);
    node.getParam("recordGtPath", recordGtPath);
    node.getParam("repeat_param_list", repeat_param_list);
    node.getParam("useGazebo", useGazebo);
    
    // student
    RepeatNode student(posesPath, recordGtPath, repeat_param_list, useGazebo);
    tf::TransformListener listener;

    // PD-control
    pdControl pd(PD_param_list);

    // velocity control
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/dingo_velocity_controller/cmd_vel", 1);
    ros::Rate rate(10.0);

    // listen the joystick
    ros::Subscriber joy = node.subscribe("/bluetooth_teleop/joy", 1, &RepeatNode::joyCallback, &student);

    // publish the initial pose for amcl
    ros::Publisher pub_init = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
    student.setInitialPose_for_amcl(pub_init, student.poses.front());

    while(node.ok())
    {   if(!student.isBegin)
            student.setInitialPose_for_amcl(pub_init, student.poses.front());
        else
        {
            tf::StampedTransform transform;
            try
            {
                listener.waitForTransform("/map","/base_link",ros::Time(0),ros::Duration(3.0));
                listener.lookupTransform("/map","/base_link",ros::Time(0),transform);
                student.repeat(transform, pd);
                pd.vel_cmd(pub, rate);
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
        
        ros::spinOnce();
        // rate.sleep();
    }
    return 0;
}