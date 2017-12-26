/**
 * @file offb_main.cpp
 * @author Julian Oes <julian@oes.ch>
 * @license BSD 3-clause
 *
 * @brief ROS node to do offboard control of PX4 through MAVROS.
 *
 * Initial code taken from http://dev.px4.io/ros-mavros-offboard.html
 */


#include <ros/ros.h>
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

mavros_msgs::State current_state, current_state2, current_state3, current_state4;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
void state_cb2(const mavros_msgs::State::ConstPtr& msg) {
    current_state2 = *msg;
}
void state_cb3(const mavros_msgs::State::ConstPtr& msg) {
    current_state3 = *msg;
}
void state_cb4(const mavros_msgs::State::ConstPtr& msg) {
    current_state4 = *msg;
}

/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */
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


/*
 * Call main using `rosrun offb offb_main`.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arming");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("drone1/mavros/state", 10, state_cb);
    ros::Subscriber state_sub2 = nh.subscribe<mavros_msgs::State>
                                ("drone2/mavros/state", 10, state_cb2);
	ros::Subscriber state_sub3 = nh.subscribe<mavros_msgs::State>
                                ("drone3/mavros/state", 10, state_cb3);
	ros::Subscriber state_sub4 = nh.subscribe<mavros_msgs::State>
                                ("drone4/mavros/state", 10, state_cb4);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("drone1/mavros/cmd/arming");
    ros::ServiceClient arming_client2 = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("drone2/mavros/cmd/arming");
	ros::ServiceClient arming_client3 = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("drone3/mavros/cmd/arming");
	ros::ServiceClient arming_client4 = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("drone4/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("drone1/mavros/set_mode");
    ros::ServiceClient set_mode_client2 = nh.serviceClient<mavros_msgs::SetMode>
                                         ("drone2/mavros/set_mode");
	ros::ServiceClient set_mode_client3 = nh.serviceClient<mavros_msgs::SetMode>
                                         ("drone3/mavros/set_mode");
	ros::ServiceClient set_mode_client4 = nh.serviceClient<mavros_msgs::SetMode>
                                         ("drone4/mavros/set_mode");
 
    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(100);

    // Wait for FCU connection.
    while (ros::ok() && current_state.connected && current_state2.connected && current_state3.connected && current_state3.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;


    while (ros::ok()) {
	
        int c = getch();
	//ROS_INFO("C: %d",c);
        if (c != EOF) {
            switch (c) {
	    	case 107:    // start arming
			{
			offb_set_mode.request.custom_mode = "OFFBOARD";
			set_mode_client.call(offb_set_mode);
			set_mode_client2.call(offb_set_mode);
			set_mode_client3.call(offb_set_mode);
			set_mode_client4.call(offb_set_mode);
			arm_cmd.request.value = true;
			arming_client.call(arm_cmd);
			arming_client2.call(arm_cmd);
			arming_client3.call(arm_cmd);
			arming_client4.call(arm_cmd);
			ROS_INFO("offboard enabled");
			ROS_INFO("start arming");
            break;
			}
			case 108:    // close arming
			{
			offb_set_mode.request.custom_mode = "MANUAL";
			set_mode_client.call(offb_set_mode);
			set_mode_client2.call(offb_set_mode);
			set_mode_client3.call(offb_set_mode);
			set_mode_client4.call(offb_set_mode);
			arm_cmd.request.value = false;
			arming_client.call(arm_cmd);
			arming_client2.call(arm_cmd);
			arming_client3.call(arm_cmd);
			arming_client4.call(arm_cmd);
			ROS_INFO("offboard disabled");
			ROS_INFO("stop arming");
            break;
			}
            case 63:
                return 0;
                break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
