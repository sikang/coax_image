#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"
#include <sensor_msgs/Image.h>
#include <cv.hpp>
#include <cxcore.hpp>
#include <coax_msgs/imgState.h>
using namespace cv;

int r = 8,initial_flag=0;
double xc=43; 
double yc=29.7,x=yc;
double theta=0,rho=xc,previous_theta=0,previous_rho=xc,previous_x=yc;
double t1=0,t2=0,dt=0;
ros::Publisher img_pub;
ros::Publisher img_state_pub;
void stateCallback(const sensor_msgs::Image::ConstPtr& msg)
{  
//	t2 = t1;
//	t1 = ros::Time::now().toSec();
//	dt = t1-t2;
//	std::cout<<dt<<std::endl;
  coax_msgs::imgState image_state;
  sensor_msgs::Image out_img;
	int h,w,theta_n=0,rho_n=0,x_n=0;
	double rho_sum=0,theta_sum=0,x_sum=0,a,b,x0,y0;
	const Mat raw_img(msg->height,msg->width,CV_8UC1,(void*)(&msg->data[0]));
	out_img.header.stamp = ros::Time::now();
	out_img.header.frame_id = "out_img";
	out_img.height = msg->height/r;
	out_img.width = msg->width/r;
	out_img.encoding = "mono8";
	out_img.is_bigendian = 0;
	out_img.step = msg->width/r;
 
	Mat resize_img,edge;
	
	vector<Vec2f> lines;
  resize(raw_img,resize_img,Size(),0.125,0.125,INTER_LINEAR);	
	Canny(resize_img,edge,150,350,3);
  HoughLines(edge,lines,1,CV_PI/180,25);

	 for(int i=0;i<lines.size();i++){
	
		theta = lines[i][1];

		if(theta>CV_PI/2){
			theta=theta-CV_PI;
		}
		if(theta>=0)
			rho = lines[i][0];
		
		else if(theta<0)
			rho = -lines[i][0];

		if(abs(theta-previous_theta)<1&&abs(rho-previous_rho)<40){
			theta_sum +=theta;
			theta_n += 1;
		
			rho_sum += rho;
			rho_n += 1;
		}
	}
	
	if(theta_n!=0){
		previous_theta = theta_sum/theta_n;
    if(rho_n!=0){
	  if(abs(previous_rho-rho_sum/rho_n)<40)
		previous_rho = rho_sum/rho_n;
		}
		else if(rho_n==0){		
			previous_rho= 40;
			}
		
		theta_sum = 0;
		rho_sum = 0;
		theta_n = 0;
		rho_n = 0;
	  if(initial_flag==0){
			initial_flag = 1;
		}
		
 
	}
	a = cos(previous_theta),b = sin(previous_theta);
	double distance = (a*xc+b*yc-previous_rho)*a; 
	x0 = a*previous_rho, y0=b*previous_rho;

	if (abs(distance)>1&&initial_flag==0){
		ROS_WARN("bad image!!");
	}
	
	image_state.y = distance;
	image_state.theta = previous_theta;
  Point pt1(cvRound(x0+1000*(-b)),
			cvRound(y0+1000*a));

	Point pt2(cvRound(x0-1000*(-b)),
			cvRound(y0-1000*a));
	line(edge,pt1,pt2,Scalar(255),3,8);


	 for(int j=0;j<lines.size();j++){
	
		theta = lines[j][1];
    x = lines[j][0];
		if(abs(theta-previous_theta-CV_PI/2)<1&&abs(x-previous_x)<30){
		
			x_sum += x;
			x_n += 1;
	}
	}
if(x_n!=0){
	  if(abs(previous_x-x_sum/x_n)<30)
	  previous_x = x_sum/x_n;
	
		x_sum = 0;
		x_n = 0;
	}
	a = cos(previous_theta+CV_PI/2),b = sin(previous_theta+CV_PI/2);
	double distance_x = (a*xc+b*yc-previous_x)*b; 
	x0 = a*previous_x, y0=b*previous_x;
	Point pt3(cvRound(x0+1000*(-b)),
			cvRound(y0+1000*a));

	Point pt4(cvRound(x0-1000*(-b)),
			cvRound(y0-1000*a));
	line(edge,pt3,pt4,Scalar(255),1,8);
  
	if(initial_flag==0){
		std::cout<<"ERROR:"<<previous_x<<"( "<<x0<<","<<y0<<")"<<std::endl;
	}

	image_state.x = distance_x;
  
	img_state_pub.publish(image_state);

  if (initial_flag==0){
		initial_flag=1;
	}
	
	for(h=0;h<out_img.height;h++)
	{
		for(w=0;w<out_img.width;w++){
			out_img.data.push_back(edge.at<char>(h,w));
		}
	}
	img_pub.publish(out_img);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"listerner1");
	ros::NodeHandle n;
	ROS_INFO("ready...");
	ros::Subscriber state_sub=n.subscribe("/image",1,stateCallback);
	img_pub=n.advertise<sensor_msgs::Image>("out_img",1);
	img_state_pub=n.advertise<coax_msgs::imgState>("img_state",1);
	ros::spin();
	return 0 ;
}

