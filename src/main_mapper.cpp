#include "include/lambdaMapper.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lambda_mapper");
	ros::NodeHandle n;

	LambdaMapper map(n);

	ros::spin();

	return 0;
}
