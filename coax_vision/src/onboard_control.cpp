#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <coax_msgs/CoaxConfigureControl.h>
#include <com/sbapi.h>
#include <OnboardControl.h>
#include <nav_msgs/Odometry.h>

#ifndef PI
#define PI 3.14159
#endif

OnboardControl::OnboardControl(ros::NodeHandle &node)
:

 reach_nav_state(node.serviceClient<coax_msgs::CoaxReachNavState>("reach_nav_state"))
,configure_comm(node.serviceClient<coax_msgs::CoaxConfigureComm>("configure_comm"))
,configure_control(node.serviceClient<coax_msgs::CoaxConfigureControl>("configure_control"))
,set_timeout(node.serviceClient<coax_msgs::CoaxSetTimeout>("set_timeout"))
,onboard_stage_pub(node.advertise<coax_msgs::stageState>("onboard_stage",1))
,LOW_POWER_DETECTED(false)
,CONTROL_MODE(CONTROL_LANDED)
,FIRST_START(false)
,FIRST_STATE(true)
,FIRST_LANDING(false)
,FIRST_HOVER(false)
,INIT_DESIRE(false)
,INIT_IMU(false)
,coax_nav_mode(0)
,coax_control_mode(0)
,coax_state_age(0)
,raw_control_age(0)
,init_count(0)
,init_imu_count(0)
,last_state_time(0.0)
{  
	set_nav_mode = node.advertiseService("set_nav_mode", &OnboardControl::setNavMode, this);
	set_control_mode = node.advertiseService("set_control_mode", &OnboardControl::setControlMode, this);
}

OnboardControl::~OnboardControl() {}

bool OnboardControl::reachNavState(int des_state, float timeout) {
	coax_msgs::CoaxReachNavState srv;
	srv.request.desiredState = des_state;
	srv.request.timeout = timeout;
	reach_nav_state.call(srv);	

	return 0;
}

bool OnboardControl::configureComm(int frequency, int contents) {
	coax_msgs::CoaxConfigureComm srv;
	srv.request.frequency = frequency;
	srv.request.contents = contents;
	configure_comm.call(srv);
	
	return 0;
}

bool OnboardControl::configureControl(size_t rollMode, size_t pitchMode, size_t yawMode, size_t altitudeMode) {
	coax_msgs::CoaxConfigureControl srv;
	srv.request.rollMode = rollMode;
	srv.request.pitchMode = pitchMode;
	srv.request.yawMode = yawMode;
	srv.request.altitudeMode = altitudeMode;
	configure_control.call(srv);
	//ROS_INFO("called configure control");
	return 0;
}

bool OnboardControl::setTimeout(size_t control_timeout_ms, size_t watchdog_timeout_ms) {
	coax_msgs::CoaxSetTimeout srv;
	srv.request.control_timeout_ms = control_timeout_ms;
	srv.request.watchdog_timeout_ms = watchdog_timeout_ms;
	set_timeout.call(srv);
	
	return 0;
}
//==============
//ServiceServer
//==============
bool OnboardControl::setNavMode(coax_vision::SetNavMode::Request &req, coax_vision::SetNavMode::Response &out) {
	out.result = 0;
	
	switch (req.mode) 
	{	
		case SB_NAV_STOP:
			break;
		case SB_NAV_IDLE:
			break;
	}
	return 0;
}

bool OnboardControl::setControlMode(coax_vision::SetControlMode::Request &req, coax_vision::SetControlMode::Response &out) {
	out.result = 0;
	
	switch (req.mode) 
	{
		case 1:
			if (CONTROL_MODE != CONTROL_LANDED){
				ROS_INFO("Start can only be executed from mode CONTROL_LANDED");
				out.result = -1;
				break;
			}

			if (coax_nav_mode != SB_NAV_RAW) {
				if (coax_nav_mode != SB_NAV_STOP) {
					reachNavState(SB_NAV_STOP, 0.5);
					ros::Duration(0.5).sleep(); // make sure CoaX is in SB_NAV_STOP mode
				}
				reachNavState(SB_NAV_RAW, 0.5);
			}
			// switch to start procedure
			INIT_DESIRE = false;
			init_count = 0;
			rotor_ready_count = 0;
			CONTROL_MODE = CONTROL_START;
      stage = 0;
			onboard_stage.stage = stage;
		  onboard_stage_pub.publish(onboard_stage);	
			break;
    case 2:
		  if(stage==-1||stage==2){
				stage = 1;
        onboard_stage.stage=stage;
				onboard_stage_pub.publish(onboard_stage);
		    break;
			}
		    out.result = -1;
		    ROS_WARN("check the state, not ready for mode HOVER");
				break;
	  case 3:
      if(stage!=1){
				out.result = -1;
				ROS_WARN("check the state,not ready for mode LOCALIZE");
				break;
			}
			stage = 2;
			break;
    
		case 4:
		if(stage ==1||stage ==2){
		  stage = 3;
		
			ROS_INFO("LANDING...");
		  break;}
			out.result = -1;
			ROS_WARN("check the state, not ready for LANDING");
			break;
	default:
			ROS_INFO("Non existent control mode request!");
			out.result = -1;		
	}
	return true;
}





int main(int argc, char **argv) {
  ros::init(argc, argv, "OnboardControl");
	ros::NodeHandle nh("~");
	OnboardControl control(nh);
	ros::Duration(1.5).sleep(); 

	control.configureComm(100, SBS_MODES | SBS_BATTERY | SBS_GYRO | SBS_ACCEL | SBS_CHANNELS | SBS_RPY | SBS_ALTITUDE_ALL |SBS_ALL);
	// control.setTimeout(500, 5000);

	control.configureControl(SB_CTRL_MANUAL, SB_CTRL_MANUAL, SB_CTRL_MANUAL, SB_CTRL_MANUAL);
	control.setTimeout(500, 5000);

	ROS_INFO("Initially Setup comm and control");

  ros::spin();
  return 0;
}



