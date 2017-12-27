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
apriltags::AprilTagDetections tags;
void apriltags_cb(const apriltags::AprilTagDetections::ConstPtr& msg){

  int i=0;


  std::cout << "get " <<i <<" id "<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone");
  ros::NodeHandle nh;

  ros::Rate loop_rate(30);
  ros::Subscriber sub = nh.subscribe("/apriltags/detections", 10, apriltags_cb);
  std::cout << "drone start " <<std::endl;
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }

}
