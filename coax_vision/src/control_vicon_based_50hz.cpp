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
,rpy(0,0,0), accel(0,0,0), gyro(0,0,0), rpyt_rc(0,0,0,0), rpyt_rc_trim(0,0,0,0)
,range_al(0.0)
,pos_z(0.0),vel_z(0.0)
,motor_up(0),motor_lo(0)
,servo_roll(0),servo_pitch(0)
,roll_trim(0),pitch_trim(0)
,motor1_des(0.0),motor2_des(0.0),servo1_des(0.0),servo2_des(0.0)
,des_yaw(0.0),yaw_rate_des(0.0)
,roll_des(0.0),roll_rate_des(0.0)
,pitch_des(0.0),pitch_rate_des(0.0)
,altitude_des(0.0)
,stage(0),initial_pos(0),des_pos_x(0.0),des_pos_y(0.0),des_pos_z(0.0)
,des_pos_x_origin(0.0),des_pos_y_origin(0.0)
{  
//	vicon_state_sub=node.subscribe("coax_57/vicon/odom",1,&CoaxVisionControl::viconCallback,this);
	set_nav_mode.push_back(node.advertiseService("set_nav_mode", &CoaxVisionControl::setNavMode, this));
	set_control_mode.push_back(node.advertiseService("set_control_mode", &CoaxVisionControl::setControlMode, this));

}

CoaxVisionControl::~CoaxVisionControl() {}

void CoaxVisionControl::loadParams(ros::NodeHandle &n) {
	n.getParam("motorconst/const1",pm.motor_const1);
	n.getParam("motorconst/const2",pm.motor_const2);
  n.getParam("rollconst/const",pm.servo1_const);
	n.getParam("pitchconst/const",pm.servo2_const);
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
	n.getParam("imageyawcontrol/proportional",pm.kp_imgyaw);
	n.getParam("imagerollcontrol/proportional",pm.kp_imgroll);
  n.getParam("desired/des_pos_z",pm.des_pos_z);
	n.getParam("desired/des_vel_z",pm.des_vel_z);
  n.getParam("desired/x_distance",pm.x_distance);
	n.getParam("desired/y_distance",pm.y_distance);
  n.getParam("desired/Dx_max",pm.Dx_max);
  n.getParam("desired/Dy_max",pm.Dy_max);
  n.getParam("inertia/Ixx",pm.Ixx);
	n.getParam("inertia/Iyy",pm.Iyy);

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

	w=vicon->pose.pose.orientation.w;
  x=vicon->pose.pose.orientation.x;
  y=vicon->pose.pose.orientation.y;
  z=vicon->pose.pose.orientation.z;
  eula_a=atan2(2*(w*x+y*z),(1-2*(x*x+y*y)));
  eula_b=asin(2*(w*y-z*x));
  eula_c=atan2(2*(w*z+x*y),(1-2*(y*y+z*z)));
	rpy << eula_a, eula_b, eula_c;
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
	
	global_x=vicon->pose.pose.position.x;
  global_y=vicon->pose.pose.position.y;
  global_z=vicon->pose.pose.position.z;

  twist_x=vicon->twist.twist.linear.x;
	twist_y=vicon->twist.twist.linear.y;
  twist_z=vicon->twist.twist.linear.z;
  
  twist_ang[0]=vicon->twist.twist.angular.x;
	twist_ang[1]=vicon->twist.twist.angular.y;
	twist_ang[2]=vicon->twist.twist.angular.z; 
  twist_ang_b = Rbw*twist_ang;
}

void CoaxVisionControl::StateCallback(const coax_msgs::CoaxState::ConstPtr & msg) {
	static int initTime = 200;
	static int initCounter = 0;

	static Eigen::Vector3f init_acc(0,0,0);
	static Eigen::Vector3f init_gyr(0,0,0);

	battery_voltage = msg->battery;
	coax_nav_mode = msg->mode.navigation;

	//	rpy << msg->roll, msg->pitch, msg->yaw; 
	//	std::cout << "RPY: \n" << rpy << std::endl;
	accel << msg->accel[0], msg->accel[1], msg->accel[2];
	gyro << msg->gyro[0], msg->gyro[1], msg->gyro[2];
	rpyt_rc << msg->rcChannel[4], msg->rcChannel[6], msg->rcChannel[2], msg->rcChannel[0];
	rpyt_rc_trim << msg->rcChannel[5], msg->rcChannel[7], msg->rcChannel[3], msg->rcChannel[1];
	range_al = msg->zfiltered;
   
 // global_z_p = global_z;
  sonar_zf = msg->zfiltered;
	sonar_z = msg->zrange;
 // previous = current;
 // current = ros::Time::now().toSec();
 // dt = (current-previous);
  //twist_z = (global_z - global_z_p)/dt;
 
  //std::cout<<dt<<" "<<global_z<<"----------- "<<global_z_p<<std::endl;

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

	raw_control.motor1 = motor_up;
	raw_control.motor2 = motor_lo;
	raw_control.servo1 = servo_roll;
	raw_control.servo2 = servo_pitch;

previous = current;
  current = ros::Time::now().toSec();
  dt = (current-previous);

 
  //std::cout<<dt<<std::endl;


	raw_control_pub.publish(raw_control);
	vicon_state.x=global_x;
	vicon_state.y=global_y;
	vicon_state.z=global_z;
	vicon_state.vz=twist_z;
	vicon_state.roll=rpy[0];
	vicon_state.pitch=rpy[1];
	vicon_state.yaw=rpy[2];
	vicon_control.u_up=u_up;
	vicon_control.u_lo=u_lo;
	vicon_control.u_roll=u_roll;
	vicon_control.u_pitch=u_pitch;
	vicon_control.zT_lo_a=zT_lo(0);
	vicon_control.zT_lo_b=zT_lo(1);
	vicon_control.zT_lo_c=zT_lo(2);
	sensor_state.pose.pose.position.z = sonar_z;
  sensor_state_pub.publish(sensor_state);
	//vicon_control_pub.publish(vicon_control);
	//vicon_state_pub.publish(vicon_state);
	return 1;
}

bool CoaxVisionControl::rotorReady(void) {
	if (rotor_ready_count < 0) return false;
	if (rotor_ready_count <= 300) 
		rotor_ready_count++;
	if (rotor_ready_count < 150) {
		motor1_des =(float) rotor_ready_count / 150 * pm.motor_const1;
		motor2_des = 0;
		return false;
	}
	else if (rotor_ready_count < 300&&rotor_ready_count>=150) {
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
		des_pos_x = global_x;
		des_pos_y = global_y;
    des_pos_x_origin = des_pos_x;
		des_pos_y_origin = des_pos_y;
	  des_vel_x = 0;
		des_vel_y = 0;
		des_vel_z = pm.des_vel_z;
		des_yaw = 0;
	  initial_pos = 1;
	}
  
	des_pos_z = rpyt_rc[3]-pm.des_pos_z;


 //std::cout<<des_pos_z<<std::endl;
}

void CoaxVisionControl::set_localize(void){
des_pos_x = des_pos_x_origin+pm.x_distance;
des_pos_y = des_pos_y_origin+pm.y_distance;
des_pos_z = rpyt_rc[3]-pm.des_pos_z;

}
void CoaxVisionControl::stabilizationControl(void) {
	double Dyaw,Dyaw_rate,yaw_control;
	double altitude_control,Daltitude, Daltitude_rate,Dx,Dy,Dx_rate,Dy_rate,C;
	double Fdes_x,Fdes_y,Fdes_z,Mdes_z,Tup,Tlo,Wdes_up,Wdes_lo;
	static double Daltitude_Int = 0,Dx_int = 0,Dy_int = 0,Dyaw_int = 0;

	Eigen::Vector2f state = getState();

	altitude_des = des_pos_z;
	Daltitude =  global_z-altitude_des;
	Daltitude_rate = twist_z-des_vel_z;
	Daltitude_Int += Daltitude;
	altitude_control = -pm.kp_altitude * Daltitude - pm.kd_altitude * Daltitude_rate - pm.ki_altitude * Daltitude_Int;
	Dx = global_x-des_pos_x;
	if(Dx>pm.Dx_max){
		Dx = pm.Dx_max;
	}
	else if(Dx<-pm.Dx_max){
		Dx = -pm.Dx_max;
	}
	Dy = global_y-des_pos_y;
	if(Dy>pm.Dy_max){
		Dy = pm.Dy_max;
    
	}
	else if(Dy<-pm.Dy_max){
		Dy = -pm.Dy_max;
	}
	Dx_rate = twist_x-des_vel_x;
	Dy_rate = twist_y-des_vel_y;
	Dx_int +=Dx;
	Dy_int +=Dy;
	Fdes_z = altitude_control+pm.m*gravity;

	Fdes_x = -pm.kp_xy*Dx-pm.kd_xy*Dx_rate-pm.ki_xy*Dx_int-pm.kd_pq*twist_ang[1]-pm.kp_pq*eula_b;
	Fdes_y = -pm.kp_xy*Dy-pm.kd_xy*Dy_rate-pm.ki_xy*Dy_int-pm.kd_pq*twist_ang[0]-pm.kp_pq*eula_a;
	Dyaw_rate = twist_ang[2];
	Dyaw = eula_c-des_yaw;
	Dyaw_int +=Dyaw;
	yaw_control = -pm.kp_yaw * Dyaw - pm.kd_yaw * Dyaw_rate-pm.ki_yaw*Dyaw_int; 
  Mdes_z = yaw_control-twist_ang_b[0]*twist_ang_b[1]*(pm.Ixx-pm.Iyy);
	  
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
  
	z_sp[2]=cos(pm.L_lo*acos(zT_lo[2]));
	if (z_sp[2]<cos(pm.theta_max)){
		z_sp[2]=pm.theta_max;
	}

	if(zT_lo[2]==1){
		z_sp[1]=0;
		z_sp[0]=0;
	}
	else if(zT_lo[2]!=1){
	z_sp[1]=zT_lo[1]*sqrt((1-z_sp[2]*z_sp[2])/(1-zT_lo[2]*zT_lo[2]));
  z_sp[0]=zT_lo[0]*sqrt((1-z_sp[2]*z_sp[2])/(1-zT_lo[2]*zT_lo[2]));
	}
  //ROS_INFO("u:[%f,%f,%f,%f]",u_up,u_lo,u_roll,u_pitch);  
	
	
	u_pitch=1/pm.theta_max*asin(z_sp[0]);
	u_roll=1/pm.theta_max*atan(-z_sp[1]/z_sp[2]);

  	motor1_des = u_up;
		motor2_des = u_lo;

    servo1_des = pm.k_roll*u_roll+pm.offset_roll+ pm.r_rc_coef * (rpyt_rc[0]+rpyt_rc_trim[0]);
		servo2_des = pm.k_pitch*u_pitch+pm.offset_pitch - pm.p_rc_coef * (rpyt_rc[1]+rpyt_rc_trim[1]);
	if(battery_voltage<=10.8){
		ROS_WARN("low battery");
	}
//	ROS_INFO("u:[%f,%f,%f,%f]",motor1_des,motor2_des,servo1_des,servo2_des);
}

void CoaxVisionControl::controlPublisher(size_t rate) {
	ros::Rate loop_rate(rate);

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
  //control.vicon_state_sub=nh.subscribe("coax_57/vicon/odom",1,&CoaxVisionControl::viconCallback,&control); 
  //control.coax_state_sub=nh.subscribe("state",1,&CoaxVisionControl::StateCallback,&control);
	// Load Params
	control.loadParams(nh);

	// make sure coax_server has enough time to start
	ros::Duration(1.5).sleep(); 

	control.configureComm(100, SBS_MODES | SBS_BATTERY | SBS_GYRO | SBS_ACCEL | SBS_CHANNELS | SBS_RPY | SBS_ALTITUDE_ALL);
	// control.setTimeout(500, 5000);

	control.configureControl(SB_CTRL_MANUAL, SB_CTRL_MANUAL, SB_CTRL_MANUAL, SB_CTRL_MANUAL);
	control.setTimeout(500, 5000);

	ROS_INFO("Initially Setup comm and control");

	int frequency = 50;
	control.controlPublisher(frequency);
 // control.coax_state_sub=nh.subscribe("state",1,&CoaxVisionControl::StateCallback,&control);
  ros::spin();
  return 0;
}



