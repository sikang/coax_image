#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"
#include "coax_msgs/CoaxRawControl.h"
#include "coax_msgs/CoaxState.h"
#include "coax_msgs/viconState.h"
#include "coax_msgs/viconControl.h"

void stateCallback(const coax_msgs::viconControl::ConstPtr& state)
{
	ROS_INFO("u_up: [%f]",state->u_up);
  //ROS_INFO("position: [%f]",state->z);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"listerner1");
	ros::NodeHandle n;
  ROS_INFO("ready...");
  ros::Subscriber state_sub=n.subscribe("vicon_control",1,stateCallback);
	ros::spin();
	return 0 ;
}
