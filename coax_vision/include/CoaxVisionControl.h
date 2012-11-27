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
#include <coax_msgs/CoaxState.h>
#include <coax_msgs/CoaxRawControl.h>
#include <coax_msgs/CoaxControl.h>
#include <coax_msgs/CoaxReachNavState.h>
#include <coax_msgs/CoaxConfigureComm.h>
#include <coax_msgs/CoaxSetTimeout.h>
#include <sstream>
#include <fstream>
#include <coax_vision/SetNavMode.h>
#include <coax_vision/SetControlMode.h>
#include <coax_msgs/viconState.h>
#include <coax_msgs/viconControl.h>
#include <VisionFeedback.h>
#include <CoaxFilter.h>

struct param {
	double motor_const1;
	double motor_const2;
	double servo1_const;
	double servo2_const;
	double yaw_coef1;
	double yaw_coef2;
	double thr_coef1;
	double thr_coef2;
	double r_rc_coef;
	double p_rc_coef;
  double k;
  
	double kT_up;
	double kT_lo;
	double kM_up;
	double kM_lo;
	double mM_up;
	double mM_lo;
	double bM_up;
	double bM_lo;
  double offset_roll;
	double offset_pitch;  
	double m;
  
	double L_lo;
	double theta_max;
	double mL_lo;
	double bL_lo;
	double kp_yaw;
	double kd_yaw;
	double ki_yaw;
	double kp_roll;
	double kd_roll;
	double kp_pitch;
	double kd_pitch;
	double kp_altitude;
	double kd_altitude;
	double ki_altitude;
	double kp_xy;
	double kd_xy;
	double ki_xy;
	double kp_pq;
	double kd_pq;
	double k_roll;
	double k_pitch;
	double kp_imgyaw;
	double kp_imgroll;
  double des_pos_z;
	double des_vel_z; 
	double des_acc_z;
	double x_distance;
	double y_distance;
	double range_base;
  double Dx_max;
	double Dy_max;
  double Ixx;
	double Iyy;
  double drifting;
  double R1;
	double R2;
	double Q1;
	double Q2;
  double Q3;
	double Q4;
	double Q5;
	double Q6;
	double noise_y;
	double noise_z;
  double center_z;
  double y_dis;
 };


class CoaxVisionControl : public VisionFeedback, public KF 
{

	public:
	CoaxVisionControl(ros::NodeHandle&);
	~CoaxVisionControl();

	void loadParams(ros::NodeHandle & n);
	bool reachNavState(int des_state, float timeout);
	bool configureComm(int frequency, int contents);
	bool configureControl(size_t rollMode, size_t pitchMode, size_t yawMode, size_t altitudeMode);
	bool setTimeout(size_t control_timeout_ms, size_t watchdog_timeout_ms);

	bool setNavMode(coax_vision::SetNavMode::Request &req, coax_vision::SetNavMode::Response &out);
	bool setControlMode(coax_vision::SetControlMode::Request &req, coax_vision::SetControlMode::Response &out);
  void viconCallback(const nav_msgs::Odometry::ConstPtr & vicon);
	void StateCallback(const coax_msgs::CoaxState::ConstPtr & msg);
  void set_hover(void);
	void set_localize(void);
	void set_landing(void);
	void stabilizationControl(void);
	void visionControl(void);
	bool rotorReady(void);
	void controlPublisher(size_t rate);
	
	bool setRawControl(double motor_up,double motor_lo, double servo_ro,double servo_pi);
 // ros::Subscriber vicon_state_sub;
  //ros::Subscriber coax_state_sub;
private:
	ros::ServiceClient reach_nav_state;
	ros::ServiceClient configure_comm;
	ros::ServiceClient configure_control;
	ros::ServiceClient set_timeout;
  
	ros::Subscriber vicon_state_sub;	
	ros::Subscriber coax_state_sub;

	ros::Publisher raw_control_pub;
	ros::Publisher vision_control_pub;
  ros::Publisher vicon_state_pub;
	ros::Publisher vicon_control_pub;
  ros::Publisher sensor_state_pub;
	std::vector<ros::ServiceServer> set_nav_mode;
	std::vector<ros::ServiceServer> set_control_mode;

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
	
	double battery_voltage;
	struct param pm;	
	Eigen::Vector3f zT_lo;
  Eigen::Vector3f z_sp;
	Eigen::Matrix3f Rwb;
	Eigen::Matrix3f Rbw;
  Eigen::Matrix3f Rp;
	Eigen::Vector3f rpy;
	Eigen::Vector3f accel;
	Eigen::Vector3f gyro;
	Eigen::Vector4f rpyt_rc;
	Eigen::Vector4f rpyt_rc_trim;
  Eigen::Vector3f twist_ang;
	Eigen::Vector3f twist_ang_w;
  double range_al;
	double gravity;

	double pos_z;
	double vel_z;

	double motor_up;
	double motor_lo;
	double servo_roll;
	double servo_pitch;
	double roll_trim;
	double pitch_trim;


	double motor1_des;
	double motor2_des;
	double servo1_des;
	double servo2_des;

	double des_yaw;
	double yaw_rate_des;
	double roll_des;
	double roll_rate_des;
	double pitch_des;
	double pitch_rate_des;
	double altitude_des;
  
	double q_w,q_x,q_y,q_z;
	double global_x;
	double global_y;
	double global_z;
	double global_z_p;
	double twist_x;
	double twist_y;
	double twist_z;
  
  double eula_a;
	double eula_b;
	double eula_c;
  
	double dt;
	double current;
	double previous;	
 

	double u_up;
	double u_lo;
  double u_roll;
	double u_pitch;
  int stage;
	int initial_pos;
	int initial_position;
  int initial_orien,flag_hrange;
	int initial_vicon;
	double initial_time;
	double initial_x,initial_y,initial_z;
	double initial_r,initial_l,y_dis,sensor_yaw;
	double initial_roll;
	double initial_pitch;
	double initial_yaw;
  double initial_eula_c;
  int initial_sonar;
  double rate_yaw;
  double rate_yaw_sum;
	int rate_yaw_n;
  double yaw_init;
	double des_pos_x;
	double des_pos_y;
	double des_pos_z;
 
   
	double des_vel_x;
	double des_vel_y;
	double des_vel_z;
	double des_acc_z;
  double des_pos_x_origin;
	double des_pos_y_origin;
  double h;
  double lag_lo;

	double sonar_z;

  double R1,R2;
	double Q1, Q2, Q3, Q4, Q5, Q6;

	double hrange_sum_r,hrange_sum_l,gravity_sum;
	double hrange_n,gravity_n;
	double grav;
  
	double nby_r,nby_l;
  double nbz;
	 
	Eigen::Vector3f orien;
	Eigen::Vector3f rate;
	Eigen::Vector3f rate_sum;
	Eigen::Vector3f left_y,right_y;
	Eigen::Vector3f state_z;
	Eigen::Matrix3f P_y_r,P_y_l,P_z;
	Eigen::Matrix3f Qy;
	Eigen::Matrix3f Qz;
  double y1,y2;
	Eigen::Vector3f hrange;

	coax_msgs::viconControl vicon_control;	
	coax_msgs::viconState vicon_state;
	nav_msgs::Odometry sensor_state; 
};


#endif
