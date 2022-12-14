#include <iostream>
#include <ros/ros.h>
#include "lane_detector.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "gps_ld");

	LaneDetector lane_detector;	
	std::cout<<"\n GPS-LD node started.\n";

	ros::spin();
	std::cout<<"\n GPS-LD node terminated.\n";

	return 0;
}
