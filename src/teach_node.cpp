#include <ros/ros.h>
#include "teach.hpp"

/*
  the functions of teach node:
  1. Running the map generalization algorithm, and then start this node
  2. for ps4, "circle" is to record, "chacha" is to save the map and exit the node
  */
int main(int argc,char** argv)
{   
    // node initialization
    ros::init(argc,argv,"teach");
    ros::NodeHandle node;

    // get the param from the launch file
    std::string recordPath, recordGtPath, recordMapPath;  // the path to store the teach_poses, teach_gt_poses, and map file
    double disThreshold, yawThreshold;  // threshold to store the teach pose
    bool useGazebo; // record poses from gazebo?
    node.getParam("recordPath", recordPath);
    node.getParam("recordGtPath", recordGtPath);
    node.getParam("recordMapPath", recordMapPath);
    node.getParam("disThreshold", disThreshold);
    node.getParam("yawThreshold", yawThreshold);
    node.getParam("useGazebo", useGazebo);

    // teacher
    Teacher teacher(recordPath, recordGtPath, recordMapPath, disThreshold, yawThreshold, useGazebo);
    tf::TransformListener listener;
    
    // listen the joystick
    ros::Subscriber joy = node.subscribe("/bluetooth_teleop/joy", 1, &Teacher::joyCallback, &teacher);

    ros::Rate rate(10.0);
    while(node.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/map","/base_link",ros::Time(0),ros::Duration(3.0));
            listener.lookupTransform("/map","/base_link",ros::Time(0),transform);
            teacher.preprocessPose(transform);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ros::spinOnce();
    }
    return 0;
}