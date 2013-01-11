#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"
#include <sensor_msgs/Image.h>
#include <cv.hpp>
#include <cxcore.hpp>
#include <coax_msgs/imgState.h>
using namespace cv;

const int r = 4;
int initial_flag=0;
double t1=0,t2=0,dt=0;
const int xc = 320/r,yc=240/r;
double yaw=0,dis_x=0,dis_y=0;
const int thresold = 10;
ros::Publisher img_pub;
ros::Publisher img_state_pub;
Mat resize_img,edge;
vector<Vec3f> circles;

void stateCallback(const sensor_msgs::Image::ConstPtr& msg)
{  
//		t1 = ros::Time::now().toSec();
  coax_msgs::imgState image_state;
	sensor_msgs::Image out_img;
  int h,w,match;
  static int x[6]={0,0,0,0,0,0};
	static int y[6]={0,0,0,0,0,0};
  static int previous_x[6] = {0,0,0,0,0,0}, current_x[6] = {0,0,0,0,0,0};
	static int previous_y[6] = {0,0,0,0,0,0}, current_y[6] = {0,0,0,0,0,0};	
	const Mat raw_img(msg->height,msg->width,CV_8UC1,(void*)(&msg->data[0]));
	out_img.header.stamp = ros::Time::now();
	out_img.header.frame_id = "out_img";
	out_img.height = msg->height/r;
	out_img.width = msg->width/r;
	out_img.encoding = "mono8";
	out_img.is_bigendian = 0;
	out_img.step = msg->width/r;


	resize(raw_img,resize_img,Size(),1.0/r,1.0/r,INTER_LINEAR);	
	Canny(resize_img,edge,100,300,3);
	HoughCircles(edge,circles,CV_HOUGH_GRADIENT,1,20,100,35);
/*
if(circles.size()==6&&initial_flag ==0){
    initial_flag = 1;
		for(int i=0;i<circles.size();i++){
			x[i] = cvRound(circles[i][0]);
			y[i] = cvRound(circles[i][1]);
			Point center(x[i],y[i]);
			//	int radius = cvRound(circles[i][2]);
			circle(edge,center,1,Scalar(255),-1,8,0);
			//		circle(edge,center,radius,Scalar(255),1,8,0);
      previous_x[i]=x[i];
			previous_y[i]=y[i];
		}
	ROS_INFO("Initialization success!");

}
*/
if(circles.size()>=2){
  double dis[6] = {0,0,0,0,0,0};
	match = 0;
		for(int i=0;i<circles.size();i++){
    x[i] = cvRound(circles[i][0]);
		y[i] = cvRound(circles[i][1]);
 	 int radius = cvRound(circles[i][2]);
    if (radius<12){
		for(int j=0;j<circles.size();j++){
			dis[j]=sqrt((x[i]-previous_x[j])*(x[i]-previous_x[j])+(y[i]-previous_y[j])*(y[i]-previous_y[j]));

			if(dis[j]<thresold&&match==0){
         current_x[j]=x[i];
				 current_y[j]=y[i];
				 match=1;
			}
		}
    if(match==0){
			ROS_WARN("bad circle detected,dis:[%f],[%f],[%f],[%f],[%f],[%f]",dis[0],dis[1],dis[2],dis[3],dis[4],dis[5]);
		}
		else if(match==1){ 
		Point center(x[i],y[i]);
    circle(edge,center,1,Scalar(255),-1,8,0);
   	circle(edge,center,radius,Scalar(255),1,8,0);
		}
		}
		else if(radius>=12){
    Point center(x[i],y[i]);
    circle(edge,center,1,Scalar(255),-1,8,0);
   	circle(edge,center,radius,Scalar(255),1,8,0);

		ROS_WARN("radius:[%i], number: [%i]",radius,circles.size());      		
	}
 }

	image_state.theta = yaw;
	img_state_pub.publish(image_state);
	
}
else if(initial_flag ==0){
 image_state.x = dis_x;
	image_state.y = dis_y;
	image_state.theta = yaw;
	img_state_pub.publish(image_state);
}	
	for(h=0;h<out_img.height;h++)
	{
		for(w=0;w<out_img.width;w++){
			out_img.data.push_back(edge.at<char>(h,w));
		}
	}
	img_pub.publish(out_img);
	//t2 = ros::Time::now().toSec();
	//	dt = t2-t1;
	//	std::cout<<dt<<std::endl;

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

