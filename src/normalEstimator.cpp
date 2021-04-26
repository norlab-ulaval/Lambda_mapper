#include "include/normalEstimator.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <cmath>

NormalEstimator::NormalEstimator(int _neighbor_radius, float _max_radius, int _min_neighbors):
	neighbor_radius(_neighbor_radius),//neighbors 
	max_radius(_max_radius),//m 
	min_neighbors(_min_neighbors) // minimum number of neighbors within the radius to perform normal estimation (>2), otherwise nan
{}

NormalEstimator::~NormalEstimator()
{}


std::vector<float> NormalEstimator::estimate_normals(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	float angle= msg->angle_min;
	float x, y, x_i, y_i;
	float s_xy, s_x, s_y, s_x2, slope;
	int n;

	std::vector<float> normals;
	normals.resize(msg->ranges.size());

	for(unsigned int i=0; 
			angle < msg->angle_max; 
			++i, angle+=msg->angle_increment)
	{
		x_i = msg->ranges[i]*std::cos(angle);
		y_i = msg->ranges[i]*std::sin(angle);

		if(msg->ranges[i] > msg->range_max or msg->ranges[i] == 0.) 
		{
			normals[i] = NAN;
			continue;
		}
		
		n=0;
		s_x=0;
		s_y=0;
		s_xy=0;
		s_x2=0;
		for(int j= (int)i - neighbor_radius; j<= (int)i + neighbor_radius; ++j)
		{
			if(j < 0 or j >= (int)msg->ranges.size())
				continue;
			
			x = msg->ranges[j]*std::cos(angle + (j-i)*msg->angle_increment);
			y = msg->ranges[j]*std::sin(angle + (j-i)*msg->angle_increment);

			if( (x-x_i)*(x-x_i) + (y-y_i)*(y-y_i) > max_radius*max_radius)
				continue;

			s_xy += x*y;
			s_x  += x;
			s_y  += y;
			s_x2 += x*x;
			++n;
		}

		slope = ( n*s_xy - s_x*s_y) / ( n*s_x2 - s_x*s_x);

		if(n >= min_neighbors)
		{
			normals[i] = std::atan(slope) - M_PI/2.; //result is in [-pi ; 0]
			if( -x_i*std::cos(normals[i]) -y_i*std::sin(normals[i]) < 0) 
				normals[i] += M_PI; //make it point toward the robot
		}
		else
			normals[i] = NAN;
	}

	return normals;
}

