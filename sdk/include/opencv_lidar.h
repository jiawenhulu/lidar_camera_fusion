#pragma once
#include <iostream>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <highgui.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "string.h"
#include <stdlib.h>


#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#define LEFTCONE 0
#define RIGHTCONE 1
#define ENDCONE 2
#define OTHER 3
#define CONEHEIGHT 110000

using namespace std;
using namespace cv;

//测量点数据结构，这个可以参考应用手册
struct scanDot {
    _u8   quality;
    float angle;
    float dist;
};


class LidarImage
{
public:
    //vector<scanDot> scan_data; //保存每扫描一周的雷达数据
    float scan_speed;
    void scanData(rplidar_response_measurement_node_t *buffer, size_t count, float frequency);
    void draw(Mat camImage,vector<scanDot> scanData);
};



extern vector<scanDot> scan_data;



