# GPS-Guided Lane Detection (GPS-LD)

A robust lane detection method that uses conventional image processing techniques for feature extraction and global route data as prior information to guide the lane detection process. The proposed method uses the route from the mission planner to determine the shape of the road in front of the vehicle. The road shape extracted from the route data is then used as a reference to cluster lane marking features and to verify the lane boundary detection results. 

## Architecture
The GPS-LD system takes image frame from Camera and current GPS location of a vehicle and heading measurements from the IMU in addition to the mission route data (sequence of GPS waypoints). Based on the mission and sensor data the lane detector outputs ego-lane boundaries.
GPS-LD has two major blocks: Feature Extraction and Lane Finding. The block diagram shown below shows the major blocks and their functional components of the GPS-LD system. 

<img src="/images/block_diagram.png" width="500" /> 

## Dependency 
The implementation is tested on Ubuntu 18 with ROS Melodic and other ROS packages
* [OpenCV 4.x](https://opencv.org/)
* [ROS Melodic](http://wiki.ros.org/melodic)
* [Novatel OEM7 driver](https://wiki.ros.org/novatel_oem7_driver)
* [Eigen-3.3.7](https://eigen.tuxfamily.org/index.php?title=Main_Page)
