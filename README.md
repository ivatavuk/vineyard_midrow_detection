# Vineyard Row Middle Detection
ROS package for detecting the middle of the passage between two rows of grapevines.

Point Cloud Library (PCL) is used to detect the border rows from a 3D LiDAR scan. 

![viv1](doc/vineyard_mid_row_gif.gif)

## Method

Method consists of:

1. Removing the pointcloud points inside the Remove Box 
2. Removing the pointcloud points outside the Keep Box
3. Detecting all the lines subject to max_angle and min_points parameters
4. Selecting right and left borders from detected lines
5. Calculating the middle line and publishing it to the middle_line topic  

## Installation

### Dependencies

PCL

### Build

Clone into a catkin workspace and build with

	catkin build vineyard_midrow_detection

## Usage

### Launching

To launch the node along with Rviz visualization:

	roslaunch vineyard_midrow_detection mid_row_detection.launch  

### Parameters
#### Reconfigurable parameters

***Line Detection*** - PCL line detection using RANSAC

	line_detection_distance_threshold - minimum distance for a point to be considered a line inlier
	line_detection_max_angle - maximum angle of a line to be considered a candidate for border selection
	line_detection_min_points - minimum number of points to perform line detection on

***Keep Box*** - points inside the Keep Box cuboid are considered for line detection

	keep_box_lower_bound_x
	keep_box_upper_bound_x
	keep_box_lower_bound_y
	keep_box_upper_bound_y
	keep_box_lower_bound_z
	keep_box_upper_bound_z

***Remove Box*** - points inside the Remove Box cuboid are removed. Used to remove irrelevant points such as vehicle parts

	remove_box_lower_bound_x
	remove_box_upper_bound_x
	remove_box_lower_bound_y
	remove_box_upper_bound_y
	remove_box_lower_bound_z
	remove_box_upper_bound_z

#### Launch file parameters

	rviz - start RVIZ (default true)
	rqt_reconfigure - start rqt_reconfigure (default true)
	ros_rate - ros rate (default 50Hz)
	input_pointcloud_topic - Pointcloud2 topic (default /rslidar_points)
