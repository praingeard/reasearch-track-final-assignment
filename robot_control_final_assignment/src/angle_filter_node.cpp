/**
* \file angle_filter_node.cpp
* \brief Filter angle ranges from a LaserScan
* \author Paul Raingeard de la Blétière
* \version 1.0
* \date 12/03/2022
* \param [in] min_angle Define the minimum angle of the chosen range.
* \param [in] max_angle Define the maximum angle of the chosen range.
* \param [in] scan_topic Define the topic name for the published scan.
* \details
*
* Subscribes to: <BR>
* ° /scan
*
* Publishes to: <BR>
* ° /scan_topic
*
*
* Description :
*
* This node takes a range as parameters and returns the laserscan /base_scan filtered on this range on a chosen topic.
* 
*
**/

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <algorithm>
#include <vector>
#include <string>

ros::Publisher pub; ///< Publisher to the scan topic.
double min_angle_scan; ///< Minimum angle of the chosen range.
double max_angle_scan; ///< Maximum angle of the chosen range.

/**
* \brief Gets the scan values and publishes the chosen range to the scan topic.
* \param msg received LaserScan msg.
*
* Gets and formats the initial scan values to select the chosen range. 
* Then creates a new LaserScan message to be sent to the selected topic. 
* 
*/

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