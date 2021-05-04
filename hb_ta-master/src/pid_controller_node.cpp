/**
 * \file pid_controller_node.cpp
 * \author Devon Morris <devonmorris1992@gmail.com>
 *
 */

#include <ros/ros.h>
#include "pid_controller.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pid_controller_node");
    hummingbird_controller::PIDController pid;
    ros::spin();
}
