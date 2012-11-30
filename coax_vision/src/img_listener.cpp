#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"
#include <sensor_msgs/Image.h>
#include <cv.hpp>
#include <cxcore.hpp>
#include <coax_msgs/imgState.h>
using namespace cv;

int r = 5;
ros::Publisher img_pub;
void stateCallback(const sensor_msgs::Image::ConstPtr& msg)
{
        sensor_msgs::Image out_img;
	int h,w;
	float theta;
	const Mat raw_img(msg->height,msg->width,CV_8UC1,(void*)(&msg->data[0]));

	out_img.header.stamp = ros::Time::now();
	out_img.header.frame_id = "out_img";
	out_img.height = msg->height/r;
	out_img.width = msg->width/r;
	out_img.encoding = "mono8";
	out_img.is_bigendian = 0;
	out_img.step = msg->width/r;
       	Mat resize_img,color_img,storage,edge;
	vector<Vec2f> lines;
/*       	for(h=0;h<msg->height/2;h++)
	{
		for(w=0;w<msg->width;w++){
			resize_raw.at<char>(h,w)=raw_img.at<char>(h,w);
		}
	}
*/	cvtColor(raw_img,color_img,CV_GRAY2BGR);

        resize(raw_img,resize_img,Size(),0.2,0.2,INTER_LINEAR);	
	Canny(resize_img,edge,100,300,3);
//	cvtColor(raw_img,color_img,CV_GRAY2BGR);
        HoughLines(edge,lines,1,CV_PI/180,40);
	 for(int i=0;i<lines.size();i++){
		float rho = lines[i][0];
		theta = lines[i][1];
		
		double a = cos(theta),b = sin(theta);
//		ROS_INFO("cos(theta): %f",a);
		double x0 = a*rho, y0=b*rho;
		Point pt1(cvRound(x0+1000*(-b)),
				cvRound(y0+1000*a));

		Point pt2(cvRound(x0-1000*(-b)),
				cvRound(y0-1000*a));
		line(edge,pt1,pt2,Scalar(255),1,8);
	}

	for(h=0;h<out_img.height;h++)
	{
		for(w=0;w<out_img.width;w++){
			out_img.data.push_back(edge.at<char>(h,w));
		}
	}
	img_pub.publish(out_img);
	//ROS_INFO("position: [%f]",state->z);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"listerner1");
	ros::NodeHandle n;
	ROS_INFO("ready...");
	ros::Subscriber state_sub=n.subscribe("/image",1,stateCallback);
	img_pub=n.advertise<sensor_msgs::Image>("out_img",1);
	ros::spin();
	return 0 ;
}

