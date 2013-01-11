#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"
#include <sensor_msgs/Image.h>
#include <cv.hpp>
#include <cxcore.hpp>
#include <coax_msgs/imgState.h>
using namespace cv;

const int r = 2;
int size;
double t1=0,t2=0,dt=0;
const int xc = 320/r,yc=240/r;
double yaw=0,dis_x=0,dis_y=0;
ros::Publisher img_pub;
ros::Publisher img_state_pub;
Mat resize_img,edge;
vector<Vec4i> hierarchy;
vector<vector<Point> > contours;
  


void stateCallback(const sensor_msgs::Image::ConstPtr& msg)
{  
//			t1 = ros::Time::now().toSec();
  coax_msgs::imgState image_state;
	sensor_msgs::Image out_img;
  int h,w;
	double dis_x,dis_y,area;
	static double x[]={0,0};
	static double y[]={0,0};
	static double yaw = 0.0;
	const Mat raw_img(msg->height,msg->width,CV_8UC1,(void*)(&msg->data[0]));
	out_img.header.stamp = ros::Time::now();
	out_img.header.frame_id = "out_img";
	out_img.height = msg->height/r;
	out_img.width = msg->width/r;
	out_img.encoding = "mono8";
	out_img.is_bigendian = 0;
	out_img.step = msg->width/r;

  Mat dst = Mat::zeros(msg->height,msg->width,CV_8UC1);
  
	resize(raw_img,resize_img,Size(),1.0/r,1.0/r,INTER_LINEAR);	
//	Canny(resize_img,edge,100,300,3);
  threshold(resize_img,edge,50,255,THRESH_BINARY_INV);
	findContours(edge,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
 
  drawContours(dst,contours,-1,124,1,8,hierarchy);
	
	if(contours.size()>=2){
		size = 0;
		for(int i=0;i<contours.size();i++){
			area=contourArea(contours[i]);
			
			if(area>900&&size<=2){
    	double length=arcLength(contours[i],1);
      double ratio=length*length/area;
		if(abs(ratio-13)<5){	
					int sum_x = 0;
				int sum_y = 0;
				for(int j=0;j<contours[i].size();j++){
					sum_x+=contours[i][j].x;
					sum_y+=contours[i][j].y;
				}
				x[size] =(float) sum_x/contours[i].size();
				y[size] =(float) sum_y/contours[i].size();
				int center_x = cvRound(x[size]);
				int center_y = cvRound(y[size]); 
				Point center(center_x,center_y);
				circle(dst,center,1,Scalar(255),-1,8,0);
				size++;
			}
		}
	}
	}
	dis_x = (double) xc-(x[0]+x[1])/2;
	dis_y = (double) yc-(y[0]+y[1])/2;
	if (y[0]<y[1]){
		double temp_x = x[0];
		double temp_y = y[0];
		x[0] = x[1];
		y[0] = y[1];
		x[1] = temp_x;
		y[1] = temp_y;
	}
	if(y[0]!=y[1]){
	yaw = atan((x[1]-x[0])/(y[0]-y[1]));
	}
	else if(y[0]==y[1]){
		yaw = 0;
	}
	image_state.x =  dis_y;
	image_state.y =  dis_x;
	image_state.theta = yaw;
	ros::Time time = ros::Time::now();
 	image_state.t = time;

	img_state_pub.publish(image_state);
	
	for(h=0;h<out_img.height;h++)
	{
		for(w=0;w<out_img.width;w++){
			out_img.data.push_back(dst.at<char>(h,w));
		}
	}
	img_pub.publish(out_img);
	//t2 = ros::Time::now().toSec();
	//dt = t2-t1;
	//std::cout<<dt<<std::endl;

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

