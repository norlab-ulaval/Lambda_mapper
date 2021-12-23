#pragma once

#include <ros/ros.h>
#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <lambda_mapper/LambdaGrid.h>
#include <lambda_mapper/MassGrid.h>
#include "normalEstimator.hpp"
#include "lambdaCell.hpp"

class LambdaMapper
{
  public:
	LambdaMapper(ros::NodeHandle &n);
	~LambdaMapper();

	void laserScan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void massGrid_callback(const lambda_mapper::MassGrid::ConstPtr& msg);
	void publish_occupancyGrid();
	void publish_lambdaGrid();

	void publish(const ros::TimerEvent&);

	void evolve_map(const ros::Time t);
		
  private:
	geometry_msgs::TransformStamped base_linkTlaser;

	float cell_size;
	int map_width;	
	int map_height;
	int robotOffsetMap;
	float angular_filter;
	float radius_filter;
	float error_region_area;
	std::array<float,2> error_region;
	float ph,pm;
	bool rotation_map;

	NormalEstimator normal_estimator;
	std::vector<std::vector<LambdaCell>> map;

	tf2::Quaternion orientation; //orientation of the map

	//ROS SPECIFIC
	ros::Subscriber sub_odom;
	ros::Subscriber sub_laserScan;
	ros::Subscriber sub_massGrid;
	ros::Publisher pub_occupancyGrid;
	ros::Publisher pub_lambdaGrid;
	ros::Publisher pub_massGrid;

	tf2_ros::Buffer tfBuff;
	tf2_ros::TransformListener listener;

	std::string sensor_frame;
	std::string odom_frame;
	std::string robot_frame;

	ros::Timer timer;
};

template<typename T>
std::int8_t sign(const T var){
	return (var > 0) - (var < 0);
}
