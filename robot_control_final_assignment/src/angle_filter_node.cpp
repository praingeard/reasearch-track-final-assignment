#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <algorithm>
#include <vector>
#include <string>

ros::Publisher pub;
double min_angle_scan;
double max_angle_scan;

void scanCallback(const sensor_msgs::LaserScan &msg)
{
    //initialize new scan
    sensor_msgs::LaserScan new_scan;
    double min_value = msg.angle_min;
    double increment = msg.angle_increment;
    int max_index = 0;
    int min_index = 0;
    bool got_min_value;

    //get minimum and maximum indexes for chosen angle range
    while(min_value < max_angle_scan){
        min_value = min_value + increment;
        if ((min_value > min_angle_scan) && (!got_min_value)){
            min_index = max_index;
            got_min_value = true;
        }
        max_index ++;
    }

    //get ranges and intensities in chosen range
    std::vector<float> ranges; 
    std::vector<float> intensities; 
    for (int i = 0; i < max_index-min_index; i++){
        ranges.push_back(msg.ranges[min_index + i]);
        intensities.push_back(msg.intensities[min_index + i]);
    }

    //update scan message with new values
    new_scan.header = msg.header;
    new_scan.angle_min = min_angle_scan;
    new_scan.angle_max = max_angle_scan;
    new_scan.angle_increment = increment;
    new_scan.range_min = *std::min_element(ranges.begin(), ranges.end());
    new_scan.range_max = *std::max_element(ranges.begin(), ranges.end());
    new_scan.intensities = intensities;
    new_scan.ranges = ranges;
    new_scan.time_increment = msg.time_increment;
    
    //publish scan
    pub.publish(new_scan);
}

int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS
    //system

    ros::init(argc, argv, "angle_filter_node");
    ros::NodeHandle nh;
    std::string out_topic;

    //default values
    min_angle_scan = 0.0;
    max_angle_scan = 1.0;
    out_topic = "scan_topic";

    //get params
    ros::param::get("~min_angle", min_angle_scan);
    ros::param::get("~max_angle", max_angle_scan);
    ros::param::get("~scan_topic", out_topic);

    //setup publisher and subscriber
    pub = nh.advertise<sensor_msgs::LaserScan>(out_topic, 1);
    ros::Subscriber sub = nh.subscribe("scan", 1, scanCallback);

    ros::spin();
    return 0;
}