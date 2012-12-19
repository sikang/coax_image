#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <coax_msgs/viconState.h>
#include <ros/ros.h>
#include <coax_msgs/CoaxState.h>
#include <coax_msgs/CoaxConfigureControl.h>
#include <coax_msgs/imgState.h>
#include <math.h>
#include <com/sbapi.h>
#include <CoaxVisionControl.h>
#include <nav_msgs/Odometry.h>

#ifndef PI
#define PI 3.14159
#endif

CoaxVisionControl::CoaxVisionControl(ros::NodeHandle &node)
:VisionFeedback(node), KF(node)

,reach_nav_state(node.serviceClient<coax_msgs::CoaxReachNavState>("reach_nav_state"))
,configure_comm(node.serviceClient<coax_msgs::CoaxConfigureComm>("configure_comm"))
,configure_control(node.serviceClient<coax_msgs::CoaxConfigureControl>("configure_control"))
,set_timeout(node.serviceClient<coax_msgs::CoaxSetTimeout>("set_timeout"))
,vicon_state_sub(node.subscribe("coax_57/vicon/odom",1,&CoaxVisionControl::viconCallback,this))
,coax_state_sub(node.subscribe("state",1, &CoaxVisionControl::StateCallback,this))
,img_state_sub(node.subscribe("image_state",1,&CoaxVisionControl::imgCallback,this))
,raw_control_pub(node.advertise<coax_msgs::CoaxRawControl>("rawcontrol",1))
,vision_control_pub(node.advertise<coax_msgs::CoaxControl>("visioncontrol",1))
,vicon_state_pub(node.advertise<coax_msgs::viconState>("vicon_state",1))
,vicon_control_pub(node.advertise<coax_msgs::viconControl>("vicon_control",1))
,sensor_state_pub(node.advertise<nav_msgs::Odometry>("sensor_state",1))
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
,rotor_ready_count(-1)
,last_state_time(0.0)
,accel(0,0,0), gyro(0,0,0), rpyt_rc(0,0,0,0), rpyt_rc_trim(0,0,0,0)
,range_al(0.0)
,pos_z(0.0),vel_z(0.0)
,global_x(0),global_y(0),global_z(0.0),twist_z(0.0)
,motor_up(0),motor_lo(0)
,servo_roll(0),servo_pitch(0)
,roll_trim(0),pitch_trim(0)
,motor1_des(0.0),motor2_des(0.0),servo1_des(0.0),servo2_des(0.0)
,des_yaw(0.0),yaw_rate_des(0.0)
,roll_des(0.0),roll_rate_des(0.0)
,pitch_des(0.0),pitch_rate_des(0.0)
,altitude_des(0.0)
,stage(0),initial_pos(0),initial_position(0),initial_orien(0),flag_hrange(0),initial_vicon(0),yaw_init(0),initial_sonar(0),rate_yaw_sum(0.0),rate_yaw_n(0),des_pos_x(0.0),des_pos_y(0.0),des_pos_z(0.04)
,des_acc_z(0.0),des_pos_x_origin(0.0),des_pos_y_origin(0.0)
,Ryaw(0.0001),hrange_sum_r(0.0),hrange_sum_l(0.0),gravity_sum(0.0),hrange_n(0),gravity_n(0),nbx(0.0),nby(0.0),nbz(0.0),img_yaw(0.0),stop_flag(0)
{  
	set_nav_mode.push_back(node.advertiseService("set_nav_mode", &CoaxVisionControl::setNavMode, this));
	set_control_mode.push_back(node.advertiseService("set_control_mode", &CoaxVisionControl::setControlMode, this));
  state_x << 0,
	           0,
						 0;
  state_y = state_x;
	state_z << 0.039,
	          0,
						0;
  state_x << 0,
	           0,
						 0;
	state_yaw<<0,
	           0,
						 0;

	P_x<<0.1,0,0,
	     0,0.1,0,
			 0,0,0.1;
  P_z = P_x;
	P_yaw = P_x;
	P_y = P_x;
  
	orien<<0,
	       0,
				 0;
  Rbw<<1,0,0,
	     0,1,0,
			 0,0,1;
  Rwb = Rbw;

	zT_lo<<0,
	       0,
				 1;
  eula_a = 0;
	eula_b = 0;
	eula_c = 0;
	hrange<<0,
	        0,
					0;
  initial_r = 0;
	initial_l = 0;
}

CoaxVisionControl::~CoaxVisionControl() {}

void CoaxVisionControl::loadParams(ros::NodeHandle &n) {
	n.getParam("motorconst/const1",pm.motor_const1);
	n.getParam("motorconst/const2",pm.motor_const2);
	n.getParam("yawcoef/coef1",pm.yaw_coef1);
	n.getParam("yawcoef/coef2",pm.yaw_coef2);
	n.getParam("throttlecoef/coef1",pm.thr_coef1);
	n.getParam("throttlecoef/coef2",pm.thr_coef2);
	n.getParam("rollrccoef/coef",pm.r_rc_coef);
	n.getParam("pitchrccoef/coef",pm.p_rc_coef);
	n.getParam("yawcontrol/proportional",pm.kp_yaw);
	n.getParam("yawcontrol/differential",pm.kd_yaw);
	n.getParam("yawcontrol/integration",pm.ki_yaw);
	n.getParam("rollcontrol/proportional",pm.kp_roll);
	n.getParam("rollcontrol/differential",pm.kd_roll);
	n.getParam("pitchcontrol/proportional",pm.kp_pitch);
	n.getParam("pitchcontrol/differential",pm.kd_pitch);
	n.getParam("altitude/base",pm.range_base); 
	n.getParam("altitudecontrol/proportional",pm.kp_altitude);
	n.getParam("altitudecontrol/differential",pm.kd_altitude);
	n.getParam("altitudecontrol/integration",pm.ki_altitude);
  n.getParam("altitudecontrol/xyproportional",pm.kp_xy);
	n.getParam("altitudecontrol/xydifferential",pm.kd_xy);
	n.getParam("altitudecontrol/xyintegration",pm.ki_xy); 
  n.getParam("altitudecontrol/kp_pq",pm.kp_pq);
  n.getParam("altitudecontrol/kd_pq",pm.kd_pq);
  n.getParam("altitudecontrol/k_roll",pm.k_roll);
	n.getParam("altitudecontrol/k_pitch",pm.k_pitch);
	n.getParam("linkage_factor/lower",pm.L_lo);
	n.getParam("max_swashplate_angle",pm.theta_max);
	n.getParam("phase_lag/slope/lower",pm.mL_lo);
	n.getParam("phase_lag/offset/lower",pm.bL_lo);
	n.getParam("mass",pm.m);
	n.getParam("thrust_factor/upper",pm.kT_up);
	n.getParam("thrust_factor/lower",pm.kT_lo);
	n.getParam("moment_factor/upper",pm.kM_up);
	n.getParam("moment_factor/lower",pm.kM_lo);
  n.getParam("speed_conversion/slope/upper",pm.mM_up);
	n.getParam("speed_conversion/slope/lower",pm.mM_lo);
	n.getParam("speed_conversion/offset/upper",pm.bM_up);
	n.getParam("speed_conversion/offset/lower",pm.bM_lo);
  n.getParam("offset/roll",pm.offset_roll);
	n.getParam("offset/pitch",pm.offset_pitch); 
  n.getParam("desired/des_pos_z",pm.des_pos_z);
	n.getParam("desired/des_vel_z",pm.des_vel_z);
  n.getParam("desired/des_acc_z",pm.des_acc_z);
	n.getParam("desired/x_distance",pm.x_distance);
	n.getParam("desired/y_distance",pm.y_distance);
  n.getParam("desired/Dx_max",pm.Dx_max);
  n.getParam("desired/Dy_max",pm.Dy_max);
  n.getParam("inertia/Ixx",pm.Ixx);
	n.getParam("inertia/Iyy",pm.Iyy);
	n.getParam("filter/R1",pm.R1);
	n.getParam("filter/R2",pm.R2);
	n.getParam("filter/Q1",pm.Q1);
  n.getParam("filter/Q2",pm.Q2);
  n.getParam("filter/Q3",pm.Q3);
	n.getParam("filter/Q4",pm.Q4);
  n.getParam("filter/Q5",pm.Q5);
  n.getParam("filter/Q6",pm.Q6);
	n.getParam("filter/Ryaw",pm.Ryaw);
  n.getParam("filter/Qyaw1",pm.Qyaw1);
	n.getParam("filter/Qyaw2",pm.Qyaw2);
	n.getParam("filter/noise_x",pm.noise_x);
	n.getParam("filter/noise_y",pm.noise_y);
	n.getParam("filter/noise_z",pm.noise_z);
	n.getParam("filter/noise_yaw",pm.noise_yaw);
  n.getParam("center/z",pm.center_z);
}

//===================
// Service Clients
//===================
	
bool CoaxVisionControl::reachNavState(int des_state, float timeout) {
	coax_msgs::CoaxReachNavState srv;
	srv.request.desiredState = des_state;
	srv.request.timeout = timeout;
	reach_nav_state.call(srv);	
	return 0;
}

bool CoaxVisionControl::configureComm(int frequency, int contents) {
	coax_msgs::CoaxConfigureComm srv;
	srv.request.frequency = frequency;
	srv.request.contents = contents;
	configure_comm.call(srv);
	
	return 0;
}

bool CoaxVisionControl::configureControl(size_t rollMode, size_t pitchMode, size_t yawMode, size_t altitudeMode) {
	coax_msgs::CoaxConfigureControl srv;
	srv.request.rollMode = rollMode;
	srv.request.pitchMode = pitchMode;
	srv.request.yawMode = yawMode;
	srv.request.altitudeMode = altitudeMode;
	configure_control.call(srv);
	//ROS_INFO("called configure control");
	return 0;
}

bool CoaxVisionControl::setTimeout(size_t control_timeout_ms, size_t watchdog_timeout_ms) {
	coax_msgs::CoaxSetTimeout srv;
	srv.request.control_timeout_ms = control_timeout_ms;
	srv.request.watchdog_timeout_ms = watchdog_timeout_ms;
	set_timeout.call(srv);
	
	return 0;
}
//==============
//ServiceServer
//==============
bool CoaxVisionControl::setNavMode(coax_vision::SetNavMode::Request &req, coax_vision::SetNavMode::Response &out) {
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

bool CoaxVisionControl::setControlMode(coax_vision::SetControlMode::Request &req, coax_vision::SetControlMode::Response &out) {
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
			motor1_des = 0;
			motor2_des = 0;
			servo1_des = 0;
			servo2_des = 0;
      stage = 0;
			break;
    case 2:
		  if(stage==-1||stage==2){
				stage = 1;
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
			initial_pos=0;
			break;
    
		case 4:
		if(stage ==1||stage ==2){
		  stage = 3;
			ROS_INFO("LANDING...");
		  break;}
			out.result = -1;
			ROS_WARN("check the state, not ready for LANDING");
			break;
		case 9:
			motor1_des = 0;
			motor2_des = 0;
			servo1_des = 0;
			servo2_des = 0;
			roll_trim = 0;
			pitch_trim = 0;
			
			reachNavState(SB_NAV_STOP, 0.5);
			
			rotor_ready_count = -1;
			CONTROL_MODE = CONTROL_LANDED;
			break;
		
		default:
			ROS_INFO("Non existent control mode request!");
			out.result = -1;		
	}
	return true;
}


//==============
// Subscriber
//==============

void CoaxVisionControl::viconCallback(const nav_msgs::Odometry::ConstPtr & vicon){

	q_w=vicon->pose.pose.orientation.w;
  q_x=vicon->pose.pose.orientation.x;
  q_y=vicon->pose.pose.orientation.y;
  q_z=vicon->pose.pose.orientation.z;
  eula_a=atan2(2*(q_w*q_x+q_y*q_z),(1-2*(q_x*q_x+q_y*q_y)));
  eula_b=asin(2*(q_w*q_y-q_z*q_x));
  eula_c=atan2(2*(q_w*q_z+q_x*q_y),(1-2*(q_y*q_y+q_z*q_z)));
/*
  Rwb(0,0)=cos(eula_c)*cos(eula_b);
	Rwb(0,1)=cos(eula_c)*sin(eula_b)*sin(eula_a)-sin(eula_c)*cos(eula_a);
	Rwb(0,2)=cos(eula_c)*sin(eula_b)*cos(eula_a)+sin(eula_c)*sin(eula_a);
	Rwb(1,0)=sin(eula_c)*cos(eula_b);
	Rwb(1,1)=sin(eula_c)*sin(eula_b)*sin(eula_a)+cos(eula_c)*cos(eula_a);
	Rwb(1,2)=sin(eula_c)*sin(eula_b)*cos(eula_a)-cos(eula_c)*sin(eula_a);
	Rwb(2,0)=-sin(eula_b);
	Rwb(2,1)=cos(eula_b)*sin(eula_a);
	Rwb(2,2)=cos(eula_b)*cos(eula_a);
	Rbw = Rwb.transpose();
*/
	global_x=vicon->pose.pose.position.x;
  global_y=vicon->pose.pose.position.y;
  global_z=vicon->pose.pose.position.z;

	if(initial_position==0){
		initial_x = global_x;
		initial_y = global_y;
		initial_z = global_z;
    initial_position = 1;
	}
  global_x = global_x-initial_x;
	global_y = global_y-initial_y;


  twist_x=vicon->twist.twist.linear.x;
	twist_y=vicon->twist.twist.linear.y;
  twist_z=vicon->twist.twist.linear.z;
/*  
  twist_ang[0]=vicon->twist.twist.angular.x;
	twist_ang[1]=vicon->twist.twist.angular.y;
	twist_ang[2]=vicon->twist.twist.angular.z; 
  twist_ang_w = Rwb*twist_ang;
*/
}
void CoaxVisionControl::imgCallback(const coax_msgs::imgState::ConstPtr & img){
	img_yaw =img->theta;
	img_y = img->y*8/589.75*state_z[0]+state_z[0]*orien[0];	
  img_x = img->x*8/589.75*state_z[0]+state_z[0]*orien[1];
	Eigen::Matrix3f I;
	Eigen::MatrixXf H(1,3),HT(3,1);
	Eigen::Vector3f Kyaw,Kx,Ky;
	double Syaw,Sx,Sy,Yyaw,Yx,Yy;
	I <<1,0,0,
	    0,1,0,
			0,0,1;
	H<<1,0,0;

	HT<<1,
	    0,
			0;
  Syaw = P_yaw(0,0)+Ryaw;
	Kyaw = P_yaw*HT/Syaw;
	Yyaw = img_yaw-state_yaw[0];
	state_yaw = state_yaw+Kyaw*Yyaw;
	P_yaw = (I-Kyaw*H)*P_yaw;

	
	Sx = P_x(0,0)+R1;
	Kx = P_x*HT/Sx;
	Yx = img_x - state_x[0];
	state_x = state_x+Kx*Yx;
	P_x = (I-Kx*H)*P_x;


	Sy = P_y(0,0)+R1;
	Ky = P_y*HT/Sy;
	Yy = img_y - state_y[0];
	state_y = state_y+Ky*Yy;
	P_y = (I-Ky*H)*P_y;

}
void CoaxVisionControl::StateCallback(const coax_msgs::CoaxState::ConstPtr & msg) {
	static int initTime = 200;
	static int initCounter = 0;

	static Eigen::Vector3f init_acc(0,0,0);
	static Eigen::Vector3f init_gyr(0,0,0);

	battery_voltage = msg->battery;
	coax_nav_mode = msg->mode.navigation;

	accel << msg->accel[0], -msg->accel[1], -msg->accel[2];
	gyro << msg->gyro[0],-msg->gyro[1], -msg->gyro[2];
	rpyt_rc << msg->rcChannel[4], msg->rcChannel[6], msg->rcChannel[2], msg->rcChannel[0];
  hrange << msg->hranges[0], msg->hranges[1], msg->hranges[2];
  
	orien[0] = msg->roll-0.01;
	orien[1] = -msg->pitch+0.02;
	orien[2] = -msg->yaw;

	if(initial_orien == 0){

		initial_roll = orien[0];
		initial_pitch = orien[1];
    initial_yaw = orien[2];
		initial_orien += 1;	
		R1 = pm.R1;
		R2 = pm.R2;
    Ryaw = pm.Ryaw;

		Qz<<pm.Q1,0,0,
			0,pm.Q2,0,
			0,0,pm.Q3;
		Qy<<pm.Q4,0,0,
			0,pm.Q5,0,
			0,0,pm.Q6;
		Qyaw<<pm.Qyaw1,0,0,
		      0,pm.Qyaw2,0,
					0,0,0;

	}
/*
if(hrange[2]<2.3){ 
	y2 = cos(state_yaw[0])*cos(orien[0])*(-0.3*hrange[2]*hrange[2]*hrange[2]+1.4638*hrange[2]*hrange[2]-2.5511*hrange[2]+1.8725)-initial_r;
	y1 = cos(state_yaw[0])*cos(orien[0])*(-0.3*hrange[1]*hrange[1]*hrange[1]+1.4638*hrange[1]*hrange[1]-2.5511*hrange[1]+1.8725)-initial_l;
}
else if(hrange[2]>=2.3){
	y2 = 0;
	y1 = 0;
}
  */
  
	orien[0] = orien[0]-initial_roll;
	orien[1] = orien[1]-initial_pitch;
	orien[2] = orien[2]-initial_yaw;
  
	if(orien[2]>PI){
		orien[2]=orien[2]-2*PI;
	}
	else if(orien[2]<-PI){
		orien[2]=orien[2]+2*PI;
	}
	
  Rwb(0,0)=cos(state_yaw[0])*cos(orien[1]);
	Rwb(0,1)=cos(state_yaw[0])*sin(orien[1])*sin(orien[0])-sin(orien[2])*cos(orien[0]);
	Rwb(0,2)=cos(state_yaw[0])*sin(orien[1])*cos(orien[0])+sin(orien[2])*sin(orien[0]);
	Rwb(1,0)=sin(state_yaw[0])*cos(orien[1]);
	Rwb(1,1)=sin(state_yaw[0])*sin(orien[1])*sin(orien[0])+cos(orien[2])*cos(orien[0]);
	Rwb(1,2)=sin(state_yaw[0])*sin(orien[1])*cos(orien[0])-cos(orien[2])*sin(orien[0]);
	Rwb(2,0)=-sin(orien[1]);
	Rwb(2,1)=cos(orien[1])*sin(orien[0]);
	Rwb(2,2)=cos(orien[1])*cos(orien[0]);
	Rbw = Rwb.transpose();
	

	rate[2] = gyro[2];
  rate[1] = gyro[1];
	rate[0] = gyro[0];
  rate = Rwb*rate;

	sonar_z = msg->zfiltered;

 if (sonar_z> 6)
 {
	 sonar_z = 0.04;
 }
 
	accel = Rwb*accel;
  grav = accel[2];
	
	if (FIRST_STATE) {
		last_state_time = ros::Time::now().toSec();
		FIRST_STATE = false;
		ROS_INFO("First Time Stamp: %f",last_state_time);
		return;
	}	

	if ((battery_voltage < 10.50) && !LOW_POWER_DETECTED){
		ROS_INFO("Battery Low!!! (%fV) Landing initialized",battery_voltage);
		LOW_POWER_DETECTED = true;
	}

	if (initCounter < initTime) {
		init_acc += accel;
		init_gyr += gyro;
		initCounter++;
	}
	else if (initCounter == initTime) {
		init_acc /= initTime;
		gravity = init_acc.norm();
		setGravity(gravity);
		ROS_INFO("IMU Calibration Done! Gravity: %f", gravity);
		initCounter++;

		setInit(msg->header.stamp);
	}
	else {
		processUpdate(accel, gyro, msg->header.stamp);
		measureUpdate(range_al);
	}
	return;
}

bool CoaxVisionControl::setRawControl(double motor1, double motor2, double servo1, double servo2) {
	coax_msgs::CoaxRawControl raw_control;

	motor_up = motor1;
	motor_lo = motor2;
	servo_roll = servo1;
	servo_pitch = servo2;

	if (motor1 > 1) 
		motor_up = 1;
	else if (motor1 < 0) 
		motor_up = 0;
	else 
		motor_up = motor1;

	if (motor2 > 1) 
		motor_lo = 1;
	else if (motor2 < 0) 
		motor_lo = 0;
	else 
		motor_lo = motor2;

	if (servo1 > 1)
		servo_roll = 1;
	else if (servo1 < -1)
		servo_roll = -1;
	else
		servo_roll = servo1;

	if (servo2 > 1)
		servo_pitch = 1;
	else if (servo2 < -1)
		servo_pitch = -1;
	else
		servo_pitch = servo2;
if(stop_flag==0){
	raw_control.motor1 = motor_up;
	raw_control.motor2 = motor_lo;
	raw_control.servo1 = servo_roll;
	raw_control.servo2 = servo_pitch;
}
if(stage == 3&&state_z[0]<0.05){
	raw_control.motor1 = 0;
	raw_control.motor2 = 0;
	raw_control.servo1 = 0;
	raw_control.servo2 = 0;
	stop_flag = 1;
}
	raw_control_pub.publish(raw_control);
	previous = current;
  current = ros::Time::now().toSec();
  dt = (current-previous);
	if(dt>10){
		dt=0.02;
	}
 
	vicon_state.x=global_x;
	vicon_state.y=global_y;
	vicon_state.z=global_z;
	vicon_state.vx=twist_x;
	vicon_state.vy=twist_y;
	vicon_state.vz=twist_z;
	vicon_state.roll=eula_a;
	vicon_state.pitch=eula_b;
	vicon_state.yaw=eula_c;
	vicon_state.state_x = state_x[0];
	vicon_state.state_y = state_y[0];
	vicon_state.state_z = state_z[0];
	vicon_state.vel_y = state_y[1];
  vicon_state.vel_x = state_x[1];
	vicon_state.vel_z = state_z[1];
	vicon_state.orien_roll = orien[0];
	vicon_state.orien_pitch = orien[1];
	vicon_state.orien_yaw = state_yaw[0];
	
	vicon_state_pub.publish(vicon_state);

	return 1;
}

bool CoaxVisionControl::rotorReady(void) {
	if (rotor_ready_count < 0) 
		 return false;
	if (rotor_ready_count <= 300) 
		rotor_ready_count++;
		if(sonar_z-0.2 >0&&initial_sonar==0){
		
			initial_sonar = 1;
		}
	if (rotor_ready_count < 150) {
		motor1_des =(float) rotor_ready_count / 150 * pm.motor_const1;
		motor2_des = 0;
		gravity_n += 1;
		gravity_sum +=grav;
/*
		if(hrange[2]<2.3){  
			y2 = cos(orien[0])*cos(state_yaw[0])*(-0.3*hrange[2]*hrange[2]*hrange[2]+1.4638*hrange[2]*hrange[2]-2.5511*hrange[2]+1.8725)-initial_r;
			y1 = cos(orien[0])*cos(state_yaw[0])*(-0.3*hrange[1]*hrange[1]*hrange[1]+1.4638*hrange[1]*hrange[1]-2.5511*hrange[1]+1.8725)-initial_l;
		}
		else if(hrange[2]>=2.3){
			y2 = 0;
			y1 = 0;
		}
    hrange_n += 1;
		hrange_sum_r += y2; 
		hrange_sum_l += y1;

*/
		return false;
	}
	else if (rotor_ready_count < 300&&rotor_ready_count>=150) {
/*
		if(flag_hrange == 0){
			initial_r = hrange_sum_r/hrange_n;
			initial_l = hrange_sum_l/hrange_n;
			flag_hrange = 1;
		}
		
	*/	
		motor1_des = pm.motor_const1;
		motor2_des = (float)(rotor_ready_count-150)/150*pm.motor_const2;
		return false;
	}
	if(stage==0){
		stage = -1;
	}
	return true;	
}

void CoaxVisionControl::set_hover(void){

	if(initial_pos==0)
	{
		des_pos_x = 0;
		des_pos_y = 0;
		des_pos_z = pm.center_z;
	  des_vel_x = 0;
		des_vel_y = 0;
		des_vel_z = 0;
	  des_acc_z = pm.des_acc_z;
		des_yaw = 0;
		h = pm.des_vel_z*pm.des_vel_z/(2*pm.des_acc_z);
	  gravity = gravity_sum/gravity_n;
		initial_pos = 1;
		initial_time = ros::Time::now().toSec();
		}
  
//	des_pos_z = rpyt_rc[3]-pm.des_pos_z;
    if(des_pos_z<pm.des_pos_z-h){
		if(des_vel_z<pm.des_vel_z){
			des_vel_z+=dt*des_acc_z;
		}
		else if(des_vel_z>=pm.des_vel_z){
			des_vel_z = pm.des_vel_z;
			des_acc_z = 0;
		}

		des_pos_z += des_vel_z*dt;
		 
		}
		else if(des_pos_z>=pm.des_pos_z-h&&des_pos_z<pm.des_pos_z){
			if(des_vel_z>0){
				des_acc_z = -pm.des_acc_z;
				des_vel_z = des_vel_z +dt*des_acc_z;
			}
			else if(des_vel_z<=0){
				des_vel_z = 0;
				des_acc_z = 0;
			}

			des_pos_z+=des_vel_z*dt;
		}

		else if(des_pos_z>pm.des_pos_z){
			des_pos_z = pm.des_pos_z;
		  des_vel_z = 0;
			des_acc_z = 0;
		}


}

void CoaxVisionControl::set_localize(void){
des_pos_x = des_pos_x_origin+pm.x_distance;
des_pos_y = des_pos_y_origin+pm.y_distance;
//des_pos_z = rpyt_rc[3]-pm.des_pos_z;

}

void CoaxVisionControl::set_landing(void){
	if(des_pos_z>0){
		if(des_vel_z>-pm.des_vel_z){
			des_acc_z = -pm.des_acc_z;
			des_vel_z = des_vel_z +dt*des_acc_z;
		}
		else if(des_vel_z<=-pm.des_vel_z){
			des_acc_z = 0;
  des_vel_z = -pm.des_vel_z;
		}
	des_pos_z = des_pos_z+dt*des_vel_z;
}

else if( des_pos_z<=0){
	des_pos_z = 0;
	des_vel_z = 0;
	des_acc_z = 0;
}
}

void CoaxVisionControl::stabilizationControl(void) {
	double Dyaw,Dyaw_rate,yaw_control;
	double altitude_control,Daltitude, Daltitude_rate,Dx,Dy,Dx_rate,Dy_rate,C;
	double Fdes_x,Fdes_y,Fdes_z,Mdes_z,Tup,Tlo,Wdes_up,Wdes_lo;
	static double Daltitude_Int = 0,Dx_int = 0,Dy_int = 0,Dyaw_int = 0;

  Eigen::Matrix3f I;
	Eigen::Matrix3f A,AT;
	Eigen::MatrixXf H(1,3);
  Eigen::MatrixXf HT(3,1);
	Eigen::Vector3f Kz;
  Eigen::Vector3f By,Bz,Bx;
  double Sz;
  double Yz;
	nby = pm.noise_y*(drand48()-0.5);
	nbz = pm.noise_z*(drand48()-0.5);
	nbx = pm.noise_x*(drand48()-0.5);
	Bz<<0.5*dt*dt*(accel[2]-gravity),
	   dt*(accel[2]-gravity),
     dt*nbz;
  Bx<<0.5*dt*dt*accel[0],
	    dt*accel[0],
			dt*nbx;
	By<<0.5*dt*dt*accel[1],
	   dt*accel[1],
	   dt*nby;
 

 	I=Eigen::Matrix3f::Identity();

	H<<1,0,0;

  HT<<1,
	     0,
			 0;

  A<<1,dt,-0.5*dt*dt,
	   0,1,-dt,
		 0,0,1;

  AT = A.transpose();

	   
 if(initial_orien == 1){
  Eigen::Matrix3f A_yaw,A_yawT;
	Eigen::Vector3f Byaw;

 	A_yaw<<1,-dt,0,
	       0,1,0,
				 0,0,0;
 
 	A_yawT<<1,0,0,
	      -dt,1,0,
				  0,0,0;

	nbyaw = pm.noise_yaw*(drand48()-0.5);		
   
	Byaw<<dt*rate[2],
	      dt*nbyaw,
				0;
	 
 
  state_yaw = A_yaw*state_yaw+Byaw;
	P_yaw = A_yaw*P_yaw*A_yawT+Qyaw;


	state_z = A*state_z+Bz;
  P_z = A*P_z*AT+Qz;
  Sz = P_z(0,0)+R2;
 	Kz = P_z*HT/Sz;
  Yz = sonar_z - state_z[0],
	state_z = state_z+Kz*Yz;
	P_z = (I-Kz*H)*P_z;

	state_x = A*state_x+Bx;
	P_x = A*P_x*AT+Qy;
/*
	Sy_r = P_y_r(0,0)+R1;
	Ky_r = P_y_r*HT/Sy_r;
	Yy_r = img_x - state_y[0];
	state_x = state_x+Ky_r*Yy_r;
	P_y_r = (I-Ky_r*H)*P_y_r;
*/
	state_y = A*state_y+By;
	P_y = A*P_y*AT+Qy;
/*
	Sy_l = P_y_l(0,0)+R1;
	Ky_l = P_y_l*HT/Sy_l;
	Yy_l = img_y - state_y[0];
	state_y = state_y+Ky_l*Yy_l;
	P_y_l = (I-Ky_l*H)*P_y_l;
*/
 
 }
 
	altitude_des = des_pos_z;
	Daltitude =  state_z[0]-altitude_des;
	Daltitude_rate = state_z[1]-des_vel_z;
	
	if (state_z[0]<0.1&&stage==1){
				Daltitude_Int = 0;
	}
	else if(state_z[0]>0.1||stage!=1){ 
	Daltitude_Int += Daltitude;
	}
	altitude_control = -pm.kp_altitude * Daltitude - pm.kd_altitude * Daltitude_rate - pm.ki_altitude * Daltitude_Int;
	Dx = state_x[0];
  
	if(Dx>pm.Dx_max){
		Dx = pm.Dx_max;
	}
	else if(Dx<-pm.Dx_max){
		Dx = -pm.Dx_max;
	}

	Dy = state_y[0];
	if(Dy>pm.Dy_max){
		Dy = pm.Dy_max;
    
	}
	else if(Dy<-pm.Dy_max){
		Dy = -pm.Dy_max;
	}
	Dx_rate = state_x[1];
	Dy_rate = state_y[1];
	Dx_int +=Dx;
	if (initial_orien == 1){
	Dy_int +=Dy;
	}
	Fdes_z = altitude_control+pm.m*(gravity+des_acc_z);

	Fdes_x = -pm.kp_xy*Dx-pm.kd_xy*Dx_rate-pm.ki_xy*Dx_int-pm.kd_pq*rate[1];//pm.kp_pq*orien[1];
	Fdes_y = -pm.kp_xy*Dy-pm.kd_xy*Dy_rate-pm.ki_xy*Dy_int-pm.kd_pq*rate[0];//-pm.kp_pq*orien[0];
	Dyaw_rate = rate[2];
	Dyaw = state_yaw[0]-des_yaw;
	Dyaw_int +=Dyaw;
	yaw_control = -pm.kp_yaw * Dyaw - pm.kd_yaw * Dyaw_rate-pm.ki_yaw*Dyaw_int; 
  Mdes_z = yaw_control-gyro[0]*gyro[1]*(pm.Ixx-pm.Iyy);
	  
	C = pm.kT_up*pm.kM_lo+pm.kT_lo*pm.kM_up;
  
	Tup=(pm.kM_lo*Fdes_z-pm.kT_lo*Mdes_z)/C;
	zT_lo[2]=sqrt(1/(1+(C*C*(Fdes_x*Fdes_x+Fdes_y*Fdes_y)/(pm.kT_lo*pm.kT_lo*(Fdes_z*pm.kM_up+Mdes_z*pm.kT_up)*(Fdes_z*pm.kM_up+Mdes_z*pm.kT_up)))));
	Tlo=(pm.kM_up*Fdes_z+pm.kT_up*Mdes_z)/(zT_lo[2]*C);

	if(Tup<0){
		Tup = 0;
	}
	if(Tlo<0){
		Tlo = 0;
	}
	Wdes_up = sqrt(Tup);
	Wdes_lo = sqrt(Tlo);
	u_up = (Wdes_up-pm.bM_up)/pm.mM_up;
	u_lo = (Wdes_lo-pm.bM_lo)/pm.mM_lo;

	if(u_up<pm.motor_const1)
	{
		u_up=pm.motor_const1;
	}
	if(u_lo<pm.motor_const2)
	{
		u_lo=pm.motor_const2;
	}



	zT_lo[0]=Fdes_x/(pm.kT_lo*Tlo);
	zT_lo[1]=Fdes_y/(pm.kT_lo*Tlo);
  
	zT_lo = Rbw*zT_lo;
/*
  lag_lo=pm.mL_lo*u_lo+pm.bL_lo;
  
	Rp(0,0)=cos(lag_lo);
	Rp(0,1)=-sin(lag_lo);
	Rp(0,2)=0;
	Rp(1,0)=sin(lag_lo);
	Rp(1,1)=cos(lag_lo);
	Rp(1,2)=0;
	Rp(2,0)=0;
	Rp(2,1)=0;
	Rp(2,2)=1;
 
  zT_lo = Rp*zT_lo;
 */ 
	z_sp[2]=cos(pm.L_lo*acos(zT_lo[2]));
	if (z_sp[2]<cos(pm.theta_max)){
		z_sp[2]=cos(pm.theta_max);
	}

	if(zT_lo[2]==1){
		z_sp[1]=0;
		z_sp[0]=0;
	}
	else if(zT_lo[2]!=1){
	z_sp[1]=zT_lo[1]*sqrt((1-z_sp[2]*z_sp[2])/(1-zT_lo[2]*zT_lo[2]));
  z_sp[0]=zT_lo[0]*sqrt((1-z_sp[2]*z_sp[2])/(1-zT_lo[2]*zT_lo[2]));
	}
	
	u_pitch=1/pm.theta_max*asin(z_sp[0]);
	u_roll=1/pm.theta_max*atan(-z_sp[1]/z_sp[2]);

  	motor1_des = u_up;
		motor2_des = u_lo;

    servo1_des = pm.k_roll*u_roll+pm.offset_roll;// pm.r_rc_coef * (rpyt_rc[0]+rpyt_rc_trim[0]);
		servo2_des = pm.k_pitch*u_pitch+pm.offset_pitch+ pm.p_rc_coef * rpyt_rc[1];

	if(battery_voltage<=10.8){
		ROS_WARN("low battery");
	}
}

void CoaxVisionControl::controlPublisher(size_t rate_t) {
	ros::Rate loop_rate(rate_t);

	while(ros::ok()) {

		if (rotorReady()) {
			if(stage==1){ 
				set_hover();
				stabilizationControl();
			}
			if(stage==2){
				set_localize();
				stabilizationControl();
			}
     if(stage==3){
				set_landing();
				stabilizationControl();
			}
			}
		setRawControl(motor1_des,motor2_des,servo1_des,servo2_des);

		ros::spinOnce();
		loop_rate.sleep();
	}
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "CoaxVisionControl");
	ros::NodeHandle nh("~");
	CoaxVisionControl control(nh);
	control.loadParams(nh);
	ros::Duration(1.5).sleep(); 

	control.configureComm(100, SBS_MODES | SBS_BATTERY | SBS_GYRO | SBS_ACCEL | SBS_CHANNELS | SBS_RPY | SBS_ALTITUDE_ALL |SBS_ALL);
	// control.setTimeout(500, 5000);

	control.configureControl(SB_CTRL_MANUAL, SB_CTRL_MANUAL, SB_CTRL_MANUAL, SB_CTRL_MANUAL);
	control.setTimeout(500, 5000);

	ROS_INFO("Initially Setup comm and control");

	int frequency = 100;
	control.controlPublisher(frequency);
  ros::spin();
  return 0;
}



