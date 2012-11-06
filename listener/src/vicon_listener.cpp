#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"
#include <nav_msgs/Odometry.h>
#include "math.h"
#include "coax_msgs/CoaxState.h"
#ifndef PI
#define PI 3.14159
#endif

struct eula{
	double a;
	double b;
	double c;
}eula;

class viconState{
	public:
	viconState(ros::NodeHandle &);
	~viconState();
  ros::Subscriber coax_state_sub;
	ros::Subscriber vicon_state_sub;
	void coaxStateCallback(const coax_msgs::CoaxState::ConstPtr &state);
	void viconStateCallback(const nav_msgs::Odometry::ConstPtr &msg);
};

viconState::viconState(ros::NodeHandle &node)
{
	coax_state_sub=node.subscribe("state",1,&viconState::coaxStateCallback,this);
	vicon_state_sub=node.subscribe("coax_57/vicon/odom",1,&viconState::viconStateCallback,this);
}
viconState::~viconState(){}

void viconState::viconStateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{ 
 double	w=msg->pose.pose.orientation.w;
 double x=msg->pose.pose.orientation.x;
 double y=msg->pose.pose.orientation.y;
 double	z=msg->pose.pose.orientation.z;
	eula.a=180/PI*atan2(2*(w*x+y*z),(1-2*(x*x+y*y)));
	eula.b=180/PI*asin(2*(w*y-z*x));
	eula.c=180/PI*atan2(2*(w*z+x*y),(1-2*(y*y+z*z)));
//	ROS_INFO("orientation.a:[%f]",eula.a);
//  ROS_INFO("orientation.b:[%f]",eula.b);
//  ROS_INFO("orientation.c:[%f]",eula.c);
 double pos_x=msg->pose.pose.position.x;
 double pos_z=msg->pose.pose.position.z;
 ROS_INFO("position.z:[%f]",pos_z);
}

void viconState::coaxStateCallback(const coax_msgs::CoaxState::ConstPtr&state)
{
	ROS_INFO("hear the coax state");
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"listerner1");
	ros::NodeHandle nh;
	viconState control(nh);

  ROS_INFO("ready...");
  control.vicon_state_sub=nh.subscribe("coax_57/vicon/odom",1,&viconState::viconStateCallback,&control);

	ros::spin();
	return 0 ;
}
