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
const int xc = 80,yc=60;
double yaw=0,dis_x=0,dis_y=0;
ros::Publisher img_pub;
ros::Publisher img_state_pub;
Mat resize_img,edge;
vector<Vec3f> circles;

void stateCallback(const sensor_msgs::Image::ConstPtr& msg)
{  
//		t1 = ros::Time::now().toSec();
  coax_msgs::imgState image_state;
	sensor_msgs::Image out_img;
  int h,w;
	int x[3]={0,0,0};
	int y[3]={0,0,0};
	
	const Mat raw_img(msg->height,msg->width,CV_8UC1,(void*)(&msg->data[0]));
	out_img.header.stamp = ros::Time::now();
	out_img.header.frame_id = "out_img";
	out_img.height = msg->height/r;
	out_img.width = msg->width/r;
	out_img.encoding = "mono8";
	out_img.is_bigendian = 0;
	out_img.step = msg->width/r;


	resize(raw_img,edge,Size(),1.0/r,1.0/r,INTER_LINEAR);	
//	Canny(resize_img,edge,100,300,3);
	HoughCircles(edge,circles,CV_HOUGH_GRADIENT,1,15,100,40);

if(circles.size()==2&&initial_flag ==0){
    initial_flag = 1;
}
else if(circles.size()==2&&initial_flag==1){
	for(int i=0;i<circles.size();i++){
    x[i] = cvRound(circles[i][0]);
		y[i] = cvRound(circles[i][1]);
		Point center(x[i],y[i]);
	//	int radius = cvRound(circles[i][2]);
    circle(edge,center,1,Scalar(255),-1,8,0);
//		circle(edge,center,radius,Scalar(255),1,8,0);
      		
	}
	dis_y = (double) -(x[0]+x[1])/2+xc;
  dis_x = (double) -(y[0]+y[1])/2+yc;
 

	if (y[0]<y[1]){
	  int temp_y = y[0];
		int temp_x = x[0];
		y[0]=y[1];
		x[0]=x[1];
		y[1]=temp_y;
		x[1]=temp_x;
	}
  yaw = atan((double)(x[1]-x[0])/(y[0]-y[1])); 

  image_state.x = dis_x;
	image_state.y = dis_y;
	image_state.theta = yaw;
	img_state_pub.publish(image_state);
	
}
else if(circles.size()>=3){
	ROS_WARN("Bad circle detection..[%i]",circles.size());
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

