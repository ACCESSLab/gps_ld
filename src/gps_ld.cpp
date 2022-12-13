#include <iostream>
#include <ros/ros.h>
#include "detector.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "gps_ld");

	Detector lane;

	ros::spin();
	return 0;
}
