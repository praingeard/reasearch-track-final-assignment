/**
 * \file ui_node.cpp
 * \brief Lets the user choose the option to control the robot.
 * \author Paul Raingeard de la Blétière
 * \version 1.0
 * \date 12/03/2022
 * \details
 *
 *
 *
 * Description :
 *
 * This node implements the 3 modes (control using move_base, control by keyboard with or without help)
 *
 *
 **/

#include "ros/ros.h"
#include <string>
#include "robot_control_final_assignment/Mode.h"

int mode = 0;
bool mode_changed;

bool modeCallback(robot_control_final_assignment::Mode::Request &req,
                  robot_control_final_assignment::Mode::Response &res)
{
    mode = req.mode;
    mode_changed = true;
    return true;
}

int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS
    // system

    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("mode", modeCallback);

    while (ros::ok)
    {
        ros::spinOnce();
        if (mode_changed)
        {
            if (mode < 1 || mode > 3)
            {
                std::cout << "mode must be between 1 and 3" << std::endl;
            }

            // starting necessary modules with konsole
            if (mode == 1)
            {
                ROS_INFO("Starting move_base console");
                system("konsole -e rosrun robot_control_final_assignment move_base_position");
            }
            else if (mode == 2)
            {
                ROS_INFO("Starting keyboard controller");
                system("konsole -e rosrun teleop_twist_keyboard teleop_twist_keyboard.py _key_timeout:=0.6");
            }
            else if (mode == 3)
            {
                ROS_INFO("Starting helping module");
                int child = fork();
                if (child == 0)
                {
                    system("konsole -e rosrun teleop_twist_keyboard teleop_twist_keyboard.py _key_timeout:=0.6");
                }
                else
                {
                    system("konsole -e rosrun robot_control_final_assignment control");
                }
            }
            mode_changed = false;
        }
    }
    return 0;
}