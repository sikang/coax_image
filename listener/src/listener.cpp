#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"
#include "coax_msgs/CoaxRawControl.h"
#include "coax_msgs/CoaxState.h"
#include "coax_msgs/viconState.h"

void rawcontrolCallback(const coax_msgs::CoaxRawControl::ConstPtr& rawcontrol)
{
	//ROS_INFO("I hear: \n");
	ROS_INFO("motor1: [%f]",rawcontrol->motor1);
	//ROS_INFO("motor2: [%f]",rawcontrol->motor2);
	//ROS_INFO("servo1: [%f]",rawcontrol->servo1);
	//ROS_INFO("servo2: [%f]",rawcontrol->servo2);
}

void stateCallback(const coax_msgs::viconState::ConstPtr& state)
{
	//ROS_INFO("position.x: [%f]",state->x);
	//ROS_INFO("position.y: [%f]",state->y);
  ROS_INFO("position: [%f]",state->z);
}
	
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Hear sth");
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"listerner1");
	ros::NodeHandle n;
	ros::Subscriber sub=n.subscribe("rawcontrol",1,rawcontrolCallback);
//  ros::Subscriber state_sub=n.subscribe("vicon_state",1,stateCallback);
//  ros::Subscriber sub=n.subscribe("chatter",1,chatterCallback);
	ros::spin();
	return 0 ;
}
