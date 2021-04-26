#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>

class NormalEstimator
{
  public:
	NormalEstimator(int _neighbor_radius=20, 
					float _max_radius=4.0f, 
					int _min_neighbors=3);
	~NormalEstimator();

	std::vector<float> estimate_normals(const sensor_msgs::LaserScan::ConstPtr& msg);

	void set_neighbor_radius(int nb){neighbor_radius = nb;}
	void set_max_radius(float mr){max_radius = mr;}
	void set_min_neighbors(int mn){min_neighbors = mn;}

	int get_neighbor_radius(){return neighbor_radius;}
	float get_max_radius(){return max_radius;}
	int get_min_neighbors(){return min_neighbors;}
	
  private:
	int neighbor_radius;
	float max_radius;
	int min_neighbors;
};
