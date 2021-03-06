#include <ros/ros.h>
#include "apriltags/AprilTagDetections.h"
#include <geometry_msgs/PoseStamped.h>

#include "geometry_msgs/Pose.h"

#include <geometry_msgs/Pose.h>
#include "mavros_msgs/PositionTarget.h"
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#define ap_gain 0.4
#define forward_gain 0.0
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
mavros_msgs::PositionTarget pst;
geometry_msgs::TwistStamped vel;
geometry_msgs::PoseStamped drone_target;
bool apriltag_detect =false;
bool landing = false;
apriltags::AprilTagDetections tags;
geometry_msgs::Pose apriltag_pose;

void apriltags_cb(const apriltags::AprilTagDetections::ConstPtr& msg){

  tags = *msg;
  if(tags.detections.size() !=0 ){

    apriltag_pose = tags.detections[0].pose;
    apriltag_detect = true;
  }else{

    apriltag_detect = false;
  }
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
geometry_msgs::PoseStamped host_mocap ,turtlebot_mocap, host_mocap_last,turtlebot_mocap_last;

void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

  host_mocap = *msg;

}

void turtlebot_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

  turtlebot_mocap = *msg;

}
float qua2eul(geometry_msgs::PoseStamped& host_mocap)
{
    float pitch,yaw,roll,qx2,qy2,qz2,qw2;
    qx2=(host_mocap.pose.orientation.x)*(host_mocap.pose.orientation.x);
    qy2=(host_mocap.pose.orientation.y)*(host_mocap.pose.orientation.y);
    qz2=(host_mocap.pose.orientation.z)*(host_mocap.pose.orientation.z);
    qw2=(host_mocap.pose.orientation.w)*(host_mocap.pose.orientation.w);
    roll = atan2(2*host_mocap.pose.orientation.z*host_mocap.pose.orientation.w+2*host_mocap.pose.orientation.x*host_mocap.pose.orientation.y , 1 - 2*qy2 - 2*qz2);

    return roll;
}
char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped* vs, float dis_x, float dis_y)
{
  float errx, erry, errz, err_roll;
  float ux, uy, uz, uroll;
  //float dis_x = 0, dis_y = -0.5;
  float local_x, local_y;

  local_x = cos(vir.roll)*dis_x+sin(vir.roll)*dis_y;
  local_y = -sin(vir.roll)*dis_x+cos(vir.roll)*dis_y;

  errx = vir.x - host_mocap.pose.position.x - local_x;
  erry = vir.y - host_mocap.pose.position.y - local_y;
  errz = vir.z - host_mocap.pose.position.z - 0;
  err_roll = vir.roll - qua2eul(host_mocap);
  if(err_roll>pi)
  err_roll = err_roll - 2*pi;
  else if(err_roll<-pi)
  err_roll = err_roll + 2*pi;

  ROS_INFO("err_roll: %.3f",err_roll);

  ux = KPx*errx;
  uy = KPy*erry;
  uz = KPz*errz;
  uroll = KProll*err_roll;

  vs->twist.linear.x = ux;
  vs->twist.linear.y = uy;
  vs->twist.linear.z = uz;
  //vs->twist.angular.z = uroll;

}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone");
  ros::NodeHandle nh;
  std::cout << "start " <<std::endl;
  ros::Subscriber sub = nh.subscribe<apriltags::AprilTagDetections>("/apriltags/detections", 10, apriltags_cb);

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
  ros::Subscriber turtlebot_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody2/pose", 10, turtlebot_pos);
  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

  ros::Publisher pst_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

  ros::Rate rate(50);

  // Wait for FCU connection.
  while (ros::ok() && current_state.connected) {
    mocap_pos_pub.publish(host_mocap);
    ros::spinOnce();
    rate.sleep();
  }

  vir target,turtlebot_v;
  target.x = 0;
  target.y = 0.0;
  target.z = 1.0;
  target.roll = 0;
  host_mocap_last.header.stamp = ros::Time::now();
  host_mocap_last.pose.position.x = target.x;
  host_mocap_last.pose.position.y = target.y;
  host_mocap_last.pose.position.z = target.z;

  pst.position.x = target.x;
  pst.position.y = target.y;
  pst.position.z = target.z;
  pst.yaw = 0;
  pst.velocity.x=0;
  pst.velocity.y=0;
  pst.velocity.z=0;
  pst.header.frame_id="fcu";
  pst.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;                                 
  pst.type_mask =  mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ ;

  vel.twist.linear.x = -1* (host_mocap.pose.position.x - target.x  );
  vel.twist.linear.y = -1* (host_mocap.pose.position.y - target.y   );
  vel.twist.linear.z = -1* (host_mocap.pose.position.z - target.z   );
  //send few setpoints before starting
 for(int i = 100; ros::ok() && i > 0; --i){
   local_vel_pub.publish(vel);
//    pst.header.stamp = ros::Time::now();
   //    pst_pub.publish(pst);
   vel.header.stamp=ros::Time::now();

    mocap_pos_pub.publish(host_mocap);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  //unlock px4
  ros::Time last_request = ros::Time::now();
  while (ros::ok()) {
  mocap_pos_pub.publish(host_mocap);
  if (
       current_state.mode != "OFFBOARD" &&
       (ros::Time::now() - last_request > ros::Duration(5.0))
     ){
          if( set_mode_client.call(offb_set_mode) &&
                  offb_set_mode.response.mode_sent) {
              ROS_INFO("Offboard enabled");
              //
          }
          last_request = ros::Time::now();
      } else {

          if (!current_state.armed &&
                  (ros::Time::now() - last_request > ros::Duration(5.0))) {
              if( arming_client.call(arm_cmd) &&
                      arm_cmd.response.success) {
                  ROS_INFO("Vehicle armed");
              }
              last_request = ros::Time::now();
          }
      }
     //keyboard control
      int c = getch();
      if (c != EOF) {
          switch (c) {
              case 65:    // key up
                  target.z += 0.05;
                  break;
              case 66:    // key down
                  target.z += -0.05;
                  break;
              case 67:    // key CW(->)
                  target.roll -= 0.03;
                  break;
              case 68:    // key CCW(<-)
                  target.roll += 0.03;
                  break;
              case 119:    // key foward
                  target.x += 0.05;
                  break;
              case 120:    // key back
                  target.x += -0.05;
                  break;
              case 97:    // key left
                  target.y += 0.05;
                  break;
              case 100:    // key right
                  target.y -= 0.05;
                  break;
              case 115:    // key right
                  {
                    target.x = 0;
                    target.y = -0.5;
                    target.z = 0;
                    target.roll = 0;
                    landing = true;
                    break;
                  }
              case 108:    // close arming
                  {
                    offb_set_mode.request.custom_mode = "MANUAL";
                    set_mode_client.call(offb_set_mode);
                    arm_cmd.request.value = false;
                    arming_client.call(arm_cmd);
                    break;
                  }
              case 63:
                  return 0;
                  break;
              }
      }


    //ROS_INFO("setpoint:  x = %.2f, y = %.2f, z = %.2f", apriltag_pose.position.x, apriltag_pose.position.y, apriltag_pose.position.z);
    std::cout<<" --- --- --- --- --- -- --"<<std::endl;
    std::cout<<std::endl;
    ROS_INFO("setpoint:  roll : %.5f\n", qua2eul(host_mocap));

    turtlebot_v.x = (turtlebot_mocap.pose.position.x - turtlebot_mocap_last.pose.position.x)/(0.01);
    turtlebot_v.y = (turtlebot_mocap.pose.position.y - turtlebot_mocap_last.pose.position.y)/(0.01);


    turtlebot_mocap_last = turtlebot_mocap;

    if(landing == true){
//       drone_target.pose.position.x = 0;
//       drone_target.pose.position.y = 0.15;
//       drone_target.pose.position.z = 0.4;

       pst.type_mask= 0;
       pst.type_mask =  mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_VX |
                        mavros_msgs::PositionTarget::IGNORE_VY |
                        mavros_msgs::PositionTarget::IGNORE_VZ ;

       pst.position.x = 0;
       pst.position.y = 0.0;
       pst.position.z = 0.28;
       pst.yaw = -1*(pi/2);

       vel.twist.linear.x = -1* (host_mocap.pose.position.x - 0   );
       vel.twist.linear.y = -1* (host_mocap.pose.position.y - 0.0   );
       vel.twist.linear.z = -1* (host_mocap.pose.position.z - 0.20   );

    }else if((apriltag_detect==true) && (landing == false)){

      pst.type_mask= 0;
      pst.type_mask =  mavros_msgs::PositionTarget::IGNORE_AFX |
                       mavros_msgs::PositionTarget::IGNORE_AFY |
                       mavros_msgs::PositionTarget::IGNORE_AFZ |
                       mavros_msgs::PositionTarget::IGNORE_PX |
                       mavros_msgs::PositionTarget::IGNORE_PY |
                       mavros_msgs::PositionTarget::IGNORE_PZ ;

      geometry_msgs::Pose  ap_global ;
      double theta = -1*qua2eul( host_mocap );
      ap_global.position.x = tags.detections[0].pose.position.x *cos(theta) - tags.detections[0].pose.position.y * sin(theta);
      ap_global.position.y = tags.detections[0].pose.position.x *sin(theta) + tags.detections[0].pose.position.y * cos(theta);

      pst.velocity.x = 1*ap_gain*(ap_global.position.x);
      pst.velocity.y = -1*ap_gain * (ap_global.position.y);
      pst.velocity.z = -1.5*(host_mocap.pose.position.z - 1.0); //desired height

      vel.twist.linear.x = 1*ap_gain*(ap_global.position.x) + forward_gain * turtlebot_v.x;
      vel.twist.linear.y = -1*ap_gain * (ap_global.position.y) + forward_gain * turtlebot_v.y;
      vel.twist.linear.z = -1.5*(host_mocap.pose.position.z - (1.0-0.105));



      ROS_INFO("velocity vx: %.5f    vy: %.5f    \n",  pst.velocity.x , pst.velocity.y);
      ROS_INFO("theta = %.5f    \n",  theta * (180/pi));
      std::cout<<std::endl;
      host_mocap_last = host_mocap;

    }else if((apriltag_detect==false)&&(landing == false)){
      //no apriltags are detedcted
      pst.type_mask= 0;
      pst.type_mask =  mavros_msgs::PositionTarget::IGNORE_AFX |
                       mavros_msgs::PositionTarget::IGNORE_AFY |
                       mavros_msgs::PositionTarget::IGNORE_AFZ |
                       mavros_msgs::PositionTarget::IGNORE_VX |
                       mavros_msgs::PositionTarget::IGNORE_VY |
                       mavros_msgs::PositionTarget::IGNORE_VZ
                         ;
      vel.twist.linear.x = -1* (host_mocap.pose.position.x - host_mocap_last.pose.position.x   );
      vel.twist.linear.y = -1* (host_mocap.pose.position.y - host_mocap_last.pose.position.y   );
      vel.twist.linear.z = -1* (host_mocap.pose.position.z - host_mocap_last.pose.position.z   );
      drone_target = host_mocap_last;
      pst.position = host_mocap_last.pose.position;
    }
    pst.header.frame_id="fcu";
    pst.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pst.header.stamp = ros::Time::now();
  //  pst_pub.publish(pst);
  //  local_pos_pub.publish(drone_target);
    vel.header.stamp = ros::Time::now();
    local_vel_pub.publish(vel);
    host_mocap.header.stamp = ros::Time::now();
    mocap_pos_pub.publish(host_mocap);
    ros::spinOnce();
    rate.sleep();
  }
}
