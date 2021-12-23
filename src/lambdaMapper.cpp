#include "include/lambdaMapper.hpp"
#include "include/bresenham.h"
#include <yaml-cpp/yaml.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

//Constructor of lambdaMapper Object
LambdaMapper::LambdaMapper(ros::NodeHandle &n):
	normal_estimator(),
	listener(tfBuff)
{
// Get all parameters from config files

	// Get cell_size parameter
	if (!n.getParam("/map/cell_size", this->cell_size)){
		ROS_ERROR("Fail to get cell_size param");
		exit(0);
	}
	
	// Get map_height parameter
	if (!n.getParam("/map/map_height", this->map_height)){
		ROS_ERROR("Fail to get map_height param");
		exit(0);
	}
	if (!(map_height & 1)){ // if there is an even number of cells, add it one to make it odd so that the robot is at the center of the cells
		map_height+= 1;
	}
	
	// Get map_width parameter
	if (!n.getParam("/map/map_width", this->map_width)){
		ROS_ERROR("Fail to get map_width param");
		exit(0);
	}
	if (!(map_width & 1)){ // if there is an even number of cells, add it one to make it odd so that the robot is at the center of the cells
		map_width+=1;
	}
	
	// Get the offset between the robot and the center of the map
	if (!n.getParam("/map/robot_offset", this->robotOffsetMap)){
		ROS_ERROR("Fail to get the robot_offset param");
		exit(0);
	}
	if (robotOffsetMap > map_width/2){
		ROS_ERROR("Robot_offset is higher than half map_width");
		exit(0);
	}

	//get grid_rotation parameter
	if (!n.getParam("/map/rotation",this->rotation_map)){
		ROS_ERROR("Fail to get grid_rotation param");
		exit(0);
	}
	
	//get lambda_max parameter
	float lambdaMax;
	if (!n.getParam("/map/lambda_max",lambdaMax)){
		ROS_ERROR("Fail to get lambda_max param");
		exit(0);
	}
	LambdaCell::set_lambda_max(lambdaMax);
	
	//get lambda_unmeasured parameter
	float lambdaUnmeasured;
	if (!n.getParam("/map/lambda_unmeasured",lambdaUnmeasured)){
		ROS_ERROR("Fail to get lambda_unmeasured param");
		exit(0);
	}
	LambdaCell::set_lambda_unmeasured(lambdaUnmeasured);

	// Plot mapper parameters
	ROS_INFO("[MAPPER]  Creating Lambda Map with parameters:");
	ROS_INFO("\t- Cell size:\t %.3f m", cell_size);
	ROS_INFO("\t- robot offset:\t %d cells", robotOffsetMap);
	ROS_INFO("\t- Map size:\t %d X %d cells, %.3f X %.3f m ", map_width, map_height, cell_size*(float)map_width, cell_size*(float)map_height);
	ROS_INFO("[MAPPER]  grid rotaion:\t %s",rotation_map ? "true" : "false");	

	// Get radius filter value	
	if (!n.getParam("/sensor/radius_filter", this->radius_filter)){
		ROS_ERROR("Fail to get radius filter value");
		exit(0);
	}

	// Get angular filter value	
	if (!n.getParam("/sensor/angular_filter", this->angular_filter)){
		ROS_ERROR("Fail to get angular filter value");
		exit(0);
	}

	// Get error_region parameters	
	if ((!n.getParam("/sensor/error_region_x", this->error_region[0])) || (!n.getParam("/sensor/error_region_y", this->error_region[1]))){
		ROS_ERROR("Fail to get error_region params");
		exit(0);
	}
	this->error_region_area = error_region[0]*error_region[1];
	LambdaCell::set_error_region_area(error_region_area);
	
	// Get pm
	if (!n.getParam("/sensor/pm", this->pm)){
		ROS_ERROR("Error get pm param");
		exit(0);
	}
	LambdaCell::set_pm(pm);

	// Get ph
	if (!n.getParam("/sensor/ph", this->ph)){
		ROS_ERROR("Fail to get ph param");
		exit(0);
	}
	LambdaCell::set_ph(ph);

	ROS_INFO("[MAPPER]  Creating sensor with parameters:");
	ROS_INFO("\t- Sensor filters: radial: %.3f m angular: %.3f rad" , radius_filter, angular_filter); 
	ROS_INFO("\t- Error region: area %.3f X %.3f m (%.3f m^2)", error_region[0], error_region[1], error_region_area); 
	ROS_INFO("\t- pm:\t %.4f", pm); 
	ROS_INFO("\t- ph:\t %.4f", ph); 

	map.resize(map_width, std::vector<LambdaCell>(map_height, LambdaCell()));	

	// Get laserScanTopic param 
	std::string laserScanTopic;
	if (!n.getParam("/topics/laserScan",laserScanTopic)){
		ROS_ERROR("Fail to get laserScanTopic param");
		exit(0);
	}

	// Get massGridTopic param 
	std::string massGridTopic;
	if (!n.getParam("/topics/massGrid", massGridTopic)){
		ROS_ERROR("Fail to get massGridTopic param");
		exit(0);
	}
	sub_laserScan = n.subscribe(laserScanTopic, 10, &LambdaMapper::laserScan_callback, this);
	sub_massGrid = n.subscribe(massGridTopic, 10, &LambdaMapper::massGrid_callback, this);
	ROS_INFO("[MAPPER]  Subscribing to topics:");
	ROS_INFO("\t- LaserScan:\t %s", laserScanTopic.c_str());
	ROS_INFO("\t- MassGrid:\t %s", massGridTopic.c_str());
	
	// Get laserScanTopic param 
	std::string occupancyGridTopic;
	if (!n.getParam("/topics/occupancyGrid",occupancyGridTopic)){
		ROS_ERROR("Fail to get occupancyGridTopic param");
		exit(0);
	}

	// Get lambdaGridTopic param 
	std::string lambdaGridTopic;
	if (!n.getParam("/topics/lambdaGrid",lambdaGridTopic)){
		ROS_ERROR("Fail to get lambdaGridTopic param");
		exit(0);
	}

	pub_occupancyGrid = n.advertise<nav_msgs::OccupancyGrid>(occupancyGridTopic, 2);
	pub_lambdaGrid    = n.advertise<lambda_mapper::LambdaGrid>(lambdaGridTopic, 2);
	ROS_INFO("[MAPPER]  Publishing to topics:");
	ROS_INFO("\t- OccGrid:\t %s \t/!\\ Only for visualisation purposes", occupancyGridTopic.c_str());
	ROS_INFO("\t- LambdaGrid:\t %s", lambdaGridTopic.c_str());
	
	// Get publish rate param 
	float rate;
	if (!n.getParam("/topics/publish_rate",rate)){
		ROS_ERROR("Fail to get publish rate param");
		exit(0);
	}
	timer = n.createTimer(ros::Duration(1./rate), &LambdaMapper::publish, this);
	ROS_INFO("[MAPPER] Publishing maps at %.1f Hz", rate);

	// Get sensor_frame param
	if (!n.getParam("/tf/lidar",this->sensor_frame)){
		ROS_ERROR("Fail to get sensor_frame param");
		exit(0);
	}
	
	// Get odom_frame param
	if (!n.getParam("/tf/odom",this->odom_frame)){
		ROS_ERROR("Fail to get odom_frame param");
		exit(0);
	}
	
	// Get robot_frame param
	if (!n.getParam("/tf/robot",this->robot_frame)){
		ROS_ERROR("Fail to get robot_frame param");
		exit(0);
	}

	// Get the transformation between the sensor and the base_link
	try{
		this->base_linkTlaser = tfBuff.lookupTransform(robot_frame, sensor_frame, ros::Time(0), ros::Duration(10.0));
	}
	catch (tf2::TransformException &ex){
		ROS_ERROR("Fail to get Transform between sensor and base_link");
		exit(0);
	}
	ROS_INFO("[MAPPER]  Listening to tf frames:");
	ROS_INFO("\t- Lidar sensor:\t %s", sensor_frame.c_str());
	ROS_INFO("\t- Odometry:\t %s", odom_frame.c_str());
	ROS_INFO("\t- Robot:\t %s", robot_frame.c_str());
	/*ROS_INFO("\t- Sensor transformation:");
	ROS_INFO("\t\t- translation x: %.2f meters", base_linkTlaser.transform.translation.x);
	ROS_INFO("\t\t- translation y: %.2f meters", base_linkTlaser.transform.translation.y);
	ROS_INFO("\t\t- translation z: %.2f meters", base_linkTlaser.transform.translation.z);*/
	
	// Get neighbor_radius param 
	int neighborRadius;
	if (!n.getParam("/normals_estimation/neighbor_radius",neighborRadius)){
		ROS_ERROR("Fail to get neighbor_radius param");
		exit(0);
	}
	normal_estimator.set_neighbor_radius(neighborRadius);
	
	// Get max_radius param 
	float maxRadius;
	if (!n.getParam("/normals_estimation/max_radius",maxRadius)){
		ROS_ERROR("Fail to get max_radius param");
		exit(0);
	}
	normal_estimator.set_max_radius(maxRadius);

	// Get min_neighbors param 
	int minNeighboors;
	if (!n.getParam("/normals_estimation/min_neighbors",minNeighboors)){
		ROS_ERROR("Fail to get min_neighbors param");
		exit(0);
	}
	normal_estimator.set_min_neighbors(minNeighboors);

	ROS_INFO("[MAPPER]  Using normals estimation with");
	ROS_INFO("\t- Neighbor radius:\t %d neighbors", normal_estimator.get_neighbor_radius());
	ROS_INFO("\t- Maximum radius:\t %.2f meters", normal_estimator.get_max_radius());
	ROS_INFO("\t- Minimum neighbors:\t %d neighbors", normal_estimator.get_min_neighbors());

	orientation.setRPY(0,0,0);
}

// Destructor of lambdaMapper object
LambdaMapper::~LambdaMapper()
{}

//*********************************************************************************************
//*****************************SUBSCRIBERS*****************************************************
//*********************************************************************************************

// Function which process the sensors data and fill the map (with hit or miss)
void LambdaMapper::laserScan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	evolve_map(msg->header.stamp);

	int x,y;
	float xf, yf, xr, yr, ar, d, r;
	float bx, by, bw, bh;
	float angle = msg->angle_min - orientation.getAngle()*sign(orientation.getAxis().getZ());
	float angle_r;
	const int half_error_region_xc = error_region[0]/2./cell_size;
	const int half_error_region_yc = error_region[1]/2./cell_size;

	std::vector<float> normals = normal_estimator.estimate_normals(msg);

	double thetaMap = M_PI/2. + std::atan((map_width/2.-robotOffsetMap)/(map_height/2.));
	for(unsigned int i=0; i < msg->ranges.size(); ++i, angle+=msg->angle_increment)
	{
		r = msg->ranges[i];
		if(r == 0 || std::isinf(r)){ //we didn't hit an obstacle
			r = msg->range_max+1.; //+1 just to be sure
		}
		if(r < radius_filter) //remove robot from scan 
			continue;
		
		// Get points in the sensor frame
		ar = msg->angle_min + i*msg->angle_increment;
		//if ar is not in [-pi/2,pi/2], discard measurement
		if( !(-angular_filter < ar && ar < angular_filter)){
			continue;
		}

		geometry_msgs::PointStamped ptSensor;
		ptSensor.header = msg->header;
		ptSensor.point.x = r*cos(ar);
		ptSensor.point.y = r*sin(ar);
		ptSensor.point.z = 0;

		// Get points in the base_link frame
		geometry_msgs::PointStamped ptBase_link;
		tf2::doTransform(ptSensor, ptBase_link, base_linkTlaser);
		xr = ptBase_link.point.x;
		yr = ptBase_link.point.y;

		r = std::sqrt(xr*xr + yr*yr); // re-process (r,angle) after the offset is removed
		angle_r = fmod(std::atan2(yr,xr) - orientation.getAngle()*sign(orientation.getAxis().getZ()), 2*M_PI);
		//angle_r = fmod(std::atan2(yr,xr) - orientation.getAngle()*sign(orientation.getAxis().getZ()), 2*M_PI);
		xf = r*cos(angle_r);
		yf = r*sin(angle_r);

		//check if point is in map
		if (fabs(angle_r) <= thetaMap || fabs(angle_r) >= 2*M_PI-thetaMap){	
			d = std::min(fabs((map_width/2.+robotOffsetMap)*cell_size / cos(angle_r)),
				     fabs(map_height*cell_size/2. / sin(angle_r)));
		}
		else{
			d = std::min(fabs((map_width/2.-robotOffsetMap)*cell_size / cos(angle_r)),
				     fabs(map_height*cell_size/2. / sin(angle_r)));
		}

		if(r >= std::min(d,msg->range_max)){ //if the point is beyond the map limit or above range limit
			xf = std::min((d - cell_size/10.), (double)(msg->range_max)) * cos(angle_r); //put the point at the limit
			yf = std::min((d - cell_size/10.), (double)(msg->range_max)) * sin(angle_r); //remove small quantity to d to avoid approximation errors outside the map
			x = map_width/2 - robotOffsetMap + std::round(xf/cell_size); // add robotOffsetMap
			y = map_height/2 + std::round(yf/cell_size); 
				
		}
		else{ //point is in the map, increment hit for each pixel in error region
			x = map_width/2 - robotOffsetMap + std::round(xf/cell_size); 	// add robotOffsetMap
			y = map_height/2 + std::round(yf/cell_size); 
				
			//find bounding box of the rotated error region
			bw = (int) ( std::abs(half_error_region_xc*std::cos(-angle_r)) +  std::abs(half_error_region_yc*std::sin(-angle_r)) );
			bh = (int) ( std::abs(half_error_region_xc*std::sin(-angle_r)) +  std::abs(half_error_region_yc*std::cos(-angle_r)) );

			for(int k=-bw; k <= bw; ++k){
				for(int l=-bh; l<=bh; ++l){
					//rotate points in error frame
					bx = k*std::cos(-angle_r) - l*std::sin(-angle_r);
					by = k*std::sin(-angle_r) + l*std::cos(-angle_r);
					
					if( x+k>=0 && x+k<map_width && y+l>=0 && y+l<map_height &&  //check if point is in map
						bx >= -half_error_region_xc && bx <= half_error_region_xc && // check if point is in error region
						by >= -half_error_region_yc && by <= half_error_region_yc){
							map[x+k][y+l].inc_hit();
							//update the normal
							map[x+k][y+l].update_normal(normals[i] - orientation.getAngle()*sign(orientation.getAxis().getZ()));
					}
				}
			}
			//put the point at the boundary of the error region
			d = error_region[0]/2.f;
			xf=(r-d)*cos(angle_r); 
			yf=(r-d)*sin(angle_r);
			x = map_width/2 - robotOffsetMap + std::round(xf/cell_size); 
			y = map_height/2 + std::round(yf/cell_size); 
		}

		 //find every intersected pixels
		 std::vector<std::array<int,2>> cells = bhm_line(map_width/2-robotOffsetMap,map_height/2,x,y);
		 for(auto c : cells){
		 	map[c[0]][c[1]].inc_miss();
		 }
	}
}
void LambdaMapper::massGrid_callback(const lambda_mapper::MassGrid::ConstPtr& msg){
	std::array<float, MASS_ARRAY_SIZE> buff;

	for(int i=0; i<map_width; ++i){
		for(int j=0; j<map_height; ++j){
			for(int k=0; k<MASS_ARRAY_SIZE; ++k){
				buff[k] = msg->masses[i + map_width * (j + map_height*k )];
			}
			map[i][j].update_mass_pdf(buff);
		}
	}

	LambdaCell::set_mass_array_step(msg->mass_step);
}

//*********************************************************************************************
//*****************************PUBLISHER*******************************************************
//*********************************************************************************************

void LambdaMapper::publish(const ros::TimerEvent&){
	//publish lambdaGrid and visualisation map 
	publish_occupancyGrid();
	publish_lambdaGrid();
}

void LambdaMapper::publish_occupancyGrid(){
	nav_msgs::OccupancyGrid grid;

	//rotate around the corner of the map
	geometry_msgs::Pose origin;
	float x = -cell_size*(float)(map_width/2.-robotOffsetMap);
	float y = -cell_size*(float)map_height/2.;
	float angle = orientation.getAngle() * sign(orientation.getAxis().getZ());
	origin.position.x = x*std::cos(angle)-y*std::sin(angle);
	origin.position.y = x*std::sin(angle)+y*std::cos(angle);
	origin.position.z = 0.;

    origin.orientation.x=orientation.x();
    origin.orientation.y=orientation.y();
    origin.orientation.z=orientation.z();
    origin.orientation.w=orientation.w();

	grid.header.stamp = ros::Time::now();
	grid.header.frame_id = robot_frame;

	grid.info.resolution = cell_size;
	grid.info.width  = map_width;
	grid.info.height = map_height;
	grid.info.origin = origin;

	grid.data.resize(map_width*map_height);
	for(int i=0; i<map_width; ++i){
		for(int j=0; j<map_height; ++j){
			grid.data[i+map_width*j] = 100-100*exp(-error_region_area*map[i][j].lambda()); 
		}
	}

	pub_occupancyGrid.publish(grid);
}

void LambdaMapper::publish_lambdaGrid(){
	lambda_mapper::LambdaGrid grid;

	//rotate around the corner of the map
	geometry_msgs::Pose origin;
	float x = -cell_size*(float)(map_width/2.+robotOffsetMap);
	float y = -cell_size*(float)map_height/2.;
	float angle=orientation.getAngle() * sign(orientation.getAxis().getZ());
	origin.position.x = x*cos(angle)-y*sin(angle);
	origin.position.y = x*sin(angle)+y*cos(angle);
	origin.position.z = 0.;

    origin.orientation.x=orientation.x();
    origin.orientation.y=orientation.y();
    origin.orientation.z=orientation.z();
    origin.orientation.w=orientation.w();

	grid.header.stamp = ros::Time::now();
	grid.header.frame_id = robot_frame;

	grid.info.resolution = cell_size;
	grid.info.width  = map_width;
	grid.info.height = map_height;
	grid.info.origin = origin;

	grid.lambdaE.resize(map_width*map_height);
	grid.lambdaUp.resize(map_width*map_height);
	grid.lambdaLow.resize(map_width*map_height);
	grid.normals.resize(map_width*map_height);
	grid.masses.resize(map_width*map_height*MASS_ARRAY_SIZE);
	for(int i=0; i<map_width; ++i){
		for(int j=0; j<map_height; ++j){
			grid.lambdaE[i+map_width*j]   = map[i][j].lambda(); 
			grid.lambdaUp[i+map_width*j]  = map[i][j].lambda_up(); 
			grid.lambdaLow[i+map_width*j] = map[i][j].lambda_low(); 
			grid.normals[i+map_width*j]   = map[i][j].normal(); 
			
			for(int k=0; k<MASS_ARRAY_SIZE; ++k)
				grid.masses[i + map_width * (j + map_height*k )] = map[i][j].pdf_mass()[k]; 
 		}
	}
	grid.mass_step = LambdaCell::get_mass_array_step();

	pub_lambdaGrid.publish(grid);
}

//*********************************************************************************************
//*****************************UTILS***********************************************************
//*********************************************************************************************

void LambdaMapper::evolve_map(const ros::Time t)
{
	static ros::Time latest_update = ros::Time(0);
	
	geometry_msgs::Point pt;
	geometry_msgs::TransformStamped transform;

	// Get the transformation of the robot_frame between the two last timestamps in relation to the odom_frame
	try{
		transform = tfBuff.lookupTransform(robot_frame,  latest_update, 
				robot_frame, t,
				odom_frame, ros::Duration(1));
	}
		catch(tf2::TransformException &e){
		ROS_WARN("%s", e.what());
		return;
	}
	
	// creation of the new map with lambda
	std::vector<std::vector<LambdaCell>> e_map;
	e_map.resize(map_width, std::vector<LambdaCell>(map_height, LambdaCell()));
	
	if (rotation_map){					// The grid of lambda rotate around the robot

		tf2::Quaternion q(transform.transform.rotation.x, transform.transform.rotation.y,
					  transform.transform.rotation.z, transform.transform.rotation.w);

		tf2::Matrix3x3 m(q);
		double roll,pitch,yaw;
		m.getRPY(roll,pitch,yaw);
		q.setRPY(0,0,yaw);
		orientation *= q.inverse();
		
		transform.transform.rotation.x = 0; 		//nullify rotation: we rotate the map instead of the cells
		transform.transform.rotation.y = 0;
		transform.transform.rotation.z = 0;
		transform.transform.rotation.w = 1;
		float x = transform.transform.translation.x;
		float y = transform.transform.translation.y;
		float angle = -orientation.getAngle() * sign(orientation.getAxis().getZ());

		transform.transform.translation.x = x*cos(angle)-y*sin(angle); //rotate the translation
		transform.transform.translation.y = x*sin(angle)+y*cos(angle);
		transform.transform.translation.z = 0.;

		int x_,y_;
		static std::array<float,2> offset = {0,0};
		for(int x=-map_width/2 + robotOffsetMap; x<= map_width/2 + robotOffsetMap; ++x){
			for(int y=-map_height/2; y<=map_height/2; ++y){
				pt.x = ((float)x + offset[0])*cell_size; // true position of the sample
				pt.y = ((float)y + offset[1])*cell_size;
				pt.z = 0;

				tf2::doTransform(pt, pt, transform);

				x_= std::round(pt.x / cell_size) + map_width/2 -robotOffsetMap;
				y_= std::round(pt.y / cell_size) + map_height/2;

				if(x_<map_width && y_<map_height && x_>=0 && y_>=0 && map[x_][y_].is_measured()){
					e_map[x - robotOffsetMap + map_width/2][y + map_height/2] = map[x_][y_];
				}
				else{ //mother cell is out of the grid
					e_map[x - robotOffsetMap + map_width/2][y + map_height/2] = LambdaCell(); // init the cell
				}
			}
		}
		//Same offset for every point
		offset[0] = pt.x / cell_size - std::round(pt.x / cell_size); 
		offset[1] = pt.y / cell_size - std::round(pt.y / cell_size); 
	}
	else{							// The lambdas evolve in the grid fixed to the robot
		int xi, yi;
		static std::vector<float> offsetAllX;
		offsetAllX.resize(map_width*map_height);
		static std::vector<float> offsetAllY;
		offsetAllY.resize(map_width*map_height);
		for(int x=-map_width/2 + robotOffsetMap; x<= map_width/2 + robotOffsetMap; x++){
			for(int y=-map_height/2; y<= map_height/2; y++){
				// interpolation of the new grid with the previous one
				//pt.x = (float)x * cell_size;
				//pt.y = (float)y * cell_size;
				pt.x = ((float)x + offsetAllX[x+map_width/2-robotOffsetMap + (y+map_height/2)*map_width])  * cell_size;
				pt.y = ((float)y + offsetAllY[x+map_width/2-robotOffsetMap + (y+map_height/2)*map_width]) * cell_size;
				pt.z = 0.;
				tf2::doTransform(pt, pt, transform);
				
				// nearest neighbour interpolation
				xi = std::round(pt.x/cell_size) + map_width/2 - robotOffsetMap;
				yi = std::round(pt.y/cell_size) + map_height/2;

				if(xi >= 0 && yi >= 0 && xi < map_width && yi < map_height && map[xi][yi].is_measured()){
					e_map[x -robotOffsetMap+ map_width/2][y + map_height/2] = map[xi][yi];
				}
				else{
					e_map[x-robotOffsetMap+map_width/2][y+map_height/2] = LambdaCell(); // init the cell
				}
				
				offsetAllX[x+map_width/2-robotOffsetMap + (y+map_height/2)*map_width] = pt.x / cell_size - std::round(pt.x / cell_size);
				offsetAllY[x+map_width/2-robotOffsetMap + (y+map_height/2)*map_width] = pt.y / cell_size - std::round(pt.y / cell_size);

			}
		}
	}
	map.assign(e_map.cbegin(), e_map.cend());
	latest_update = t;
}
