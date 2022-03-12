/**
* \file move_base_position_node.cpp
* \brief Moves the robot to a selected goal (x,y).
* \author Paul Raingeard de la Blétière
* \version 1.0
* \date 12/03/2022
* \details
*
* Publishes to: <BR>
* ° /move_base/cancel
*
*
* Description :
*
* This node publishes the desired goal on the action server and stops the goal if it was not reached after the timeout (30s).
* 
*
**/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; ///<Move Base Client.

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_position_node");
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    double x = 0.;
    double y = 0.;
    while (ros::ok)
    {
        ROS_INFO("Enter the point without separator( x y ) : ");
        bool valid = false;

        while (!valid)
        {
            valid = true; //Assume the cin will be a float.
            std::cin >> x >> y;

            if (std::cin.fail()) //cin.fail() checks to see if the value in the cin
                            //stream is the correct type, if not it returns true,
                            //false otherwise.
            {
                std::cin.clear();  //This corrects the stream.
                std::cin.ignore(); //This skips the left over stream data.
                std::cout << "Please enter floats only" << std::endl;
                valid = false;
            }
        }
        ROS_INFO("you have chosen %f,%f", x, y);

        //we'll send a goal to the robot to move to (x,y)
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.orientation.w = 1.;
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);
        ROS_INFO("goal sent");

        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        //get state is robot has arrived
        if (finished_before_timeout)
        {
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("Hooray, the base moved to your goal");
            else
                ROS_INFO("The base failed to move to the goal for some reason");
        }
        //tell user that timeout has finished and cancel goal
        else
        {
            ROS_INFO("Timeout exceeded, canceling goal");
            auto msg = actionlib_msgs::GoalID();
            pub.publish(msg);
        }
    }

    return 0;
}