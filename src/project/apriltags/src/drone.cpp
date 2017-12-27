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

#define pi 3.1415926
int flag=0;
float KPx = 1;
float KPy = 1;
float KPz = 1;
float KProll = 1;

using namespace std;
struct virtual_leader
{
    float roll;
    float x;
    float y;
    float z;
};
typedef virtual_leader vir;


apriltags::AprilTagDetections tags;
void apriltags_cb(const apriltags::AprilTagDetections::ConstPtr& msg){

  tags = *msg;
  if(tags.detections.size() !=0 ){
    std::cout <<tags.detections[0].pose.position.x
              <<tags.detections[0].pose.position.y
              <<tags.detections[0].pose.position.z<<std::endl;
  }else{
    std::cout<<  "empty" <<std::endl;
  }
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
geometry_msgs::PoseStamped host_mocap;
void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

  host_mocap = *msg;

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone");
  ros::NodeHandle nh;

  ros::Rate loop_rate(30);
  ros::Subscriber sub = nh.subscribe("/apriltags/detections", 10, apriltags_cb);
  std::cout << "start " <<std::endl;
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                              ("/mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                 ("/mavros/setpoint_position/local", 10);
  ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                 ("/mavros/mocap/pose", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                     ("/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                       ("/mavros/set_mode");
  ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody1/pose", 10, host_pos);

  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }

}
