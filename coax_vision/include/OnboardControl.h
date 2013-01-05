#ifndef __COAX_VISION_CONTROL__
#define __COAX_VISION_CONTROL__

#define CONTROL_LANDED 0 // State when helicopter has landed successfully                           
#define CONTROL_START 1 // Start / Takeoff                                                          
#define CONTROL_TRIM 2 // Trim Servos                                                               
#define CONTROL_HOVER 3 // Hover                                                                    
#define CONTROL_GOTOPOS 4 // Go to Position                                                         
#define CONTROL_TRAJECTORY 5 // Follow Trajectory                                                   
#define CONTROL_LANDING 6 // Landing maneuver

#include <Eigen/Dense>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <coax_msgs/CoaxRawControl.h>
#include <coax_msgs/CoaxControl.h>
#include <coax_msgs/CoaxReachNavState.h>
#include <coax_msgs/CoaxConfigureComm.h>
#include <coax_msgs/CoaxSetTimeout.h>
#include <coax_vision/SetNavMode.h>
#include <coax_vision/SetControlMode.h>


class OnboardControl 
{

	public:
	OnboardControl(ros::NodeHandle&);
	~OnboardControl();

	bool reachNavState(int des_state, float timeout);
	bool configureComm(int frequency, int contents);
	bool configureControl(size_t rollMode, size_t pitchMode, size_t yawMode, size_t altitudeMode);
	bool setTimeout(size_t control_timeout_ms, size_t watchdog_timeout_ms);

	bool setNavMode(coax_vision::SetNavMode::Request &req, coax_vision::SetNavMode::Response &out);
	bool setControlMode(coax_vision::SetControlMode::Request &req, coax_vision::SetControlMode::Response &out);
	
private:
	ros::ServiceClient reach_nav_state;
	ros::ServiceClient configure_comm;
	ros::ServiceClient configure_control;
	ros::ServiceClient set_timeout;
  
	ros::ServiceServer set_nav_mode;
	ros::ServiceServer set_control_mode;

  ros::Publisher onboard_stage_pub;
	bool LOW_POWER_DETECTED;

	int CONTROL_MODE;
	bool FIRST_START;
	bool FIRST_STATE;
	bool FIRST_LANDING;
	bool FIRST_HOVER;
	bool INIT_DESIRE;
	bool INIT_IMU;

	bool coax_nav_mode;
	bool coax_control_mode;
	int coax_state_age;
	int raw_control_age;
	int init_count;
	int init_imu_count;
	int rotor_ready_count;
	double last_state_time;
  int stage;	
	double battery_voltage;
};


#endif
