#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"
#include <iostream>
#include <string>
#include <vector>

std::string picLoc = "/home/ubuntu/Desktop/Bowl_Movements/";

int main(int argc, char **argv)
{
	ros::init(argc, argv, "master_node");
  	ros::NodeHandle n;
	double t = ros::Time::now().toSec();
	int imageCounter = 1;
	cv::VideoCapture cap;
	// open the default camera, use something different from 0 otherwise;
	// Check VideoCapture documentation.
	if (!cap.open(0))
		return 0;
	while(ros::ok())
	{
		ros::spinOnce();
		cv::Mat frame;
		cap >> frame;
		if (frame.empty()) break; // end of video stream

		cv::imshow("Gray image", frame);

		t = (ros::Time::now().toSec() - t);
		if (t > 30) {
			continue;
		}
		ROS_WARN("Picture Taken");
		cv::imwrite(picLoc + std::to_string(imageCounter++) + ".jpg" , frame);

		if (cv::waitKey(10) == 27) break; // stop capturing by pressing ESC 
	}

	cap.release();

	return 0;
}