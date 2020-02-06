#include "opencv_lidar.h"
#define PI 3.141592653
#define RECT_HEIGHT 160000  //140000
#define RECT_WIDTH 120000   //112000


float a[9] = {-7.2,-4.4,264.8,-0.021,-1.9,1454,-0.00043,-0.0135,1};




//turn orignal lidardata to distance and angle
void LidarImage::scanData(rplidar_response_measurement_node_t *buffer, size_t count, float frequency)
{
    scan_data.clear();
    for (int pos = 0; pos < (int)count; ++pos) {
        scanDot dot;
        if (!buffer[pos].distance_q2) continue;

        dot.quality = (buffer[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        dot.angle = (buffer[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
        dot.dist = buffer[pos].distance_q2 / 4.0f;
        scan_data.push_back(dot);
    }

    scan_speed = frequency;
}

void LidarImage::draw(Mat camImage, vector<scanDot> scanData)
{
    int x = 0, y = 0, u = 0, v = 0;
    double theta = 0, rho = 0, lambda = 0;


    //draw lidarpoint
    for (int i = 0; i < scanData.size(); i++)
    {
        scanDot dot;
        dot = scanData[i];

        if(dot.angle > 120 & dot.angle < 240)
       {
			theta = dot.angle*PI / 180;
			rho = dot.dist;
			u = (int)(rho*sin(theta));
			v = (int)(rho*cos(theta));

			lambda = u*a[6] + v*a[7] + a[8];
			x = int((u*a[0] + v*a[1] + a[2]) / lambda);
			y = int((u*a[3] + v*a[4] + a[5])  / lambda);

			//ROS_INFO(":[%d,%d]", u, v);
			circle(camImage, Point(x, y), 1, Scalar(255, 255, 0), -1, 8, 0);//draw lidarpoint
       }
    }

    imshow("fusion",camImage);

}


