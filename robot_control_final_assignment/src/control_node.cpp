
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <algorithm>
#include <vector>
#include <string>
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"

//setup publisher and scans
ros::Publisher pub;
double scan_l;
double scan_r;
double scan_f;
double dist_stop = 0.5;
geometry_msgs::Twist speed;

//get all scan messages
void scanLeftCallback(const sensor_msgs::LaserScan &msg)
{
    scan_l = msg.range_min;
}

void scanRightCallback(const sensor_msgs::LaserScan &msg)
{
    scan_r = msg.range_min;
}
void scanFrontCallback(const sensor_msgs::LaserScan &msg)
{
    scan_f = msg.range_min;
}

//get speed sent by teleop keyboard
void speedCallback(const geometry_msgs::Twist &msg)
{
    speed = msg;
}

//function for robot control
void timerCallback(const ros::TimerEvent &event)
{
    //if front wall is too close, cannot go straight
    if (speed.linear.x > 0. && scan_f < dist_stop)
    {
        speed.linear.x = 0.;
        ROS_INFO("cannot go straight, wall ahead, dist is %f", scan_f);
    }
    //if left wall is too close, cannot go left
    if (speed.angular.z > 0. && scan_l < dist_stop)
    {
        speed.angular.z = 0.;
        ROS_INFO("cannot go left, wall ahead, dist is %f", scan_l);
    }
    //if right wall is too close, cannot go right
    if (speed.angular.z < 0. && scan_r < dist_stop)
    {
        speed.angular.z = 0.;
        ROS_INFO("cannot go right, wall ahead, dist is %f", scan_r);
    }
    //publish corrected speed
    pub.publish(speed);
}

int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS
    //system

    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    ros::param::get("~dist_stop", dist_stop);

    //initialize publisher and subscribers
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber sub1 = nh.subscribe("scan_left", 1, scanLeftCallback);
    ros::Subscriber sub2 = nh.subscribe("scan_right", 1, scanRightCallback);
    ros::Subscriber sub3 = nh.subscribe("scan_front", 1, scanFrontCallback);
    ros::Subscriber sub_speed = nh.subscribe("cmd_vel", 1, speedCallback);

    //initialize timer
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), timerCallback);

    ros::spin();
    return 0;
}