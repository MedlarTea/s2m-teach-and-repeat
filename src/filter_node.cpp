#include <vector>
#include <string>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <laser_teach_repeat/LaserScanWithLegConfi.h>

using namespace std;
class FilterScan
{
public:
    float threshold;
    ros::Subscriber scan_sub;
    ros::Publisher filtered_scan_pub;

    FilterScan(ros::NodeHandle *nh)
    {   
        // nh->param<float>("threshold", threshold, 0.5);
        threshold=1.5;
        ROS_INFO("Threshold:   %f", threshold);
        scan_sub = nh->subscribe<laser_teach_repeat::LaserScanWithLegConfi>("/scan_confidences_nopeople", 1, &FilterScan::scanCallback, this);
        filtered_scan_pub = nh->advertise<sensor_msgs::LaserScan>("/front/scan_filtered", 1);
    }
    void scanCallback(const laser_teach_repeat::LaserScanWithLegConfi::ConstPtr &msg);
};

void FilterScan::scanCallback(const laser_teach_repeat::LaserScanWithLegConfi::ConstPtr &msg)
{
    sensor_msgs::LaserScan filtered_scan;
    // Same informations
    filtered_scan.header = msg->header;
    filtered_scan.angle_min = msg->angle_min;
    filtered_scan.angle_max = msg->angle_max;
    filtered_scan.angle_increment = msg->angle_increment;
    filtered_scan.time_increment = msg->time_increment;
    filtered_scan.scan_time = msg->scan_time;
    filtered_scan.range_min = msg->range_min;
    filtered_scan.range_max = msg->range_max;

    // Filter the points by leg confidences
    for (size_t i=0; i<msg->ranges.size(); i++)
    {   
        // it's not a leg
        if (msg->confidences[i] < threshold)
            filtered_scan.ranges.push_back(msg->ranges[i]);
        else
            filtered_scan.ranges.push_back(msg->range_max);  // set to range_max will be filtered by gmapping
        filtered_scan.intensities.push_back(msg->intensities[i]);
    }
    filtered_scan_pub.publish(filtered_scan);
}

int main(int argc,char** argv)
{   
    ros::init(argc, argv, "filter");

    ROS_INFO("Filter scan started.");

    ros::NodeHandle nh;
    FilterScan filterScan(&nh);
    while(!ros::isShuttingDown())
    {
        //rate.sleep();
        ros::spinOnce();
    }

    return 0;
}