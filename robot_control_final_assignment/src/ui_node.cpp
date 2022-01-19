#include "ros/ros.h"
#include <string>

ros::Publisher pub;
int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS
    //system

    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    //initialize service clients
    int input = 0;

    while (ros::ok)
    {
        //getting input
        ROS_INFO("Enter 1 to enter coordinates, 2 to control robot using keyboard and 3 to control the robot with help");
        std::string str;
        std::cin >> str;

        //errors
        try
        {
            input = std::stoi(str);
        }
        catch (const std::invalid_argument &error)
        {
            ROS_ERROR("Invalid input");
        }
        if (input < 1 || input > 3)
        {
            std::cout << "input must be between 1 and 3" << std::endl;
        }

        //starting necessary modules with konsole
        if (input == 1)
        {
            ROS_INFO("Starting move_base console");
            system("konsole -e rosrun robot_control_final_assignment move_base_position");
        }
        else if (input == 2)
        {
            ROS_INFO("Starting keyboard controller");
            system("konsole -e rosrun teleop_twist_keyboard teleop_twist_keyboard.py _key_timeout:=0.6");
        }
        else if (input == 3)
        {
            ROS_INFO("Starting helping module");
            int child = fork();
            if (child ==0){
            system("konsole -e rosrun teleop_twist_keyboard teleop_twist_keyboard.py _key_timeout:=0.6");
            }
            else{
            system("konsole -e rosrun robot_control_final_assignment control");
            }
        }
    }
    return 0;
}