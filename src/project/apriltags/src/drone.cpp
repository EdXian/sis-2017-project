#include <ros/ros.h>
#include "apriltags/AprilTagDetections.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
