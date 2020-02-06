#include<ros/ros.h> 
#include<iostream> 
#include<cv_bridge/cv_bridge.h> 
#include<sensor_msgs/image_encodings.h> 
#include<image_transport/image_transport.h> 
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include "std_srvs/Empty.h"
#include "rplidar.h"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include "opencv_lidar.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)
#define PI 3.141592653

using namespace rp::standalone::rplidar;
using namespace cv;
using namespace std;

RPlidarDriver * drv = NULL;
vector<scanDot> scan_data;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int lidarcount = 0;
    lidarcount = int(scan->scan_time / scan->time_increment);
    scan_data.clear();
    for(int i = 0; i < lidarcount; i++) {
        scanDot dot;
        if(scan->ranges[i] < 10)
        {
	        dot.angle =180 -RAD2DEG(scan->angle_min + scan->angle_increment * i);
	        dot.dist = scan->ranges[i]*1000;
	        scan_data.push_back(dot);
	        //ROS_INFO(": [%f, %f]", dot.angle, scan->ranges[i]);
        }            
    }        
} 

int main( int argc, char **argv )
{
	ros::init( argc, argv, "lidar_camera" );
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
	Mat img_frame;
	LidarImage lidarImage;


	cv::VideoCapture capture( 1 ); //笔电写1，板子写0
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub_image = it.advertise( "/camera", 1 );
	if( not capture.isOpened() )
	{
	ROS_ERROR_STREAM(
	  "Failed to open camera with index " << 0 << "!"
	);
	ros::shutdown();
	}

	while( ros::ok() ) {
		capture >> img_frame;
		clock_t start, end;
		start = clock();
		if( img_frame.empty() )
		{
		    ROS_ERROR_STREAM( "Failed to capture frame!" );
		    ros::shutdown();
		}
		
		lidarImage.draw(img_frame,scan_data);

		
		sensor_msgs::ImagePtr msg_frame = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_frame).toImageMsg();

		pub_image.publish(msg_frame);

		//imshow("video",img_frame);
		end = clock();
		//ROS_INFO("info,%f", (float)(end - start) / CLOCKS_PER_SEC);

		cv::waitKey( 3 );
		ros::spinOnce();
  }

  capture.release(); 
  return EXIT_SUCCESS;
}




