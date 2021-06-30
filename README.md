# Lambda mapper

## 1. Description
Lambda mapper is a ros package allowing to perform an occupancy grid using the Lambda-Field theory.
See  [Lambda-Field: A Continuous Counterpart of the Bayesian Occupancy Grid for Risk Assessment](https://arxiv.org/abs/1903.02285) and [Lambda-Field: A Continuous Counterpart Of The Bayesian Occupancy Grid For Risk Assessment And Safe Navigation](https://arxiv.org/abs/2011.08045) for a explanation of the different parameters of the mapper.

Be aware that it is a research code that is still in progress.
Feel free to contact us if you have any question!

## 2. Hardware Requirements
To use lambda\_mapper, you need a mobile robot that provides odometry data and is equipped with a horizontally-mounted, fixed, laser range-finder.

## 3. Example
Play a simulation with one of our rosbag (mapping of a wire fence):

`roslaunch lambda_mapper map.launch`

## 4. Nodes
### 4.1 lambdaMapper
The lambdaMapper node takes in sensor messages (`sensor_msgs/LaserScan`) and a massGrid (custom message `msg/MassGrid`) to build a map (custom message `msg/lambdaGrid`).
The map can be retrieve via a ROS topic or service.
#### 4.1.1 Subscribed topics
tf (tf2\_msgs/TFMessages)
> Transforms necessary to relate frames for `laser`, `base_link` and `odometry`.

laserScan (sensor\_msgs/LaserScan)  
> Laser scan to create the map from.

massGrid (navs\_msgs/OccupancyGrid)  
>Get the mass data of the grid from this topic.
If not provided, the masses of the obstacles are supposed to be infinite everywhere.

#### 4.1.2 Published topics
lambdaGrid (msg/lambdaGrid)  
>Get the map data from this topic, which is latched, and updated periodically

occupancyGrid (navs\_msgs/OccupancyGrid) 
>Get the map data from this topic, which is latched, and updated periodically

### 4.2 Parameters
![schema Image](image\_explicative\_h.png) 
#### map namespace  
>__cell\_size__: dimension in meter of the lambda cell in the map  

>__map\_width__: number of cells along the width of the map centered on the robot  

>__map\_height__: number of cells along the height of the map centered on the robot  

>__robot\_offset__: number of cells between the center of the map and the robot frame  

>__rotation__: true: make the grid rotate around the robot frame; false: the obstacles rotates in the map (lead to deformations in case of large, repeated rotations)

>__lambda\_max__: maximum value that a lambda can take in the map. 1e+5 by default

>__lambda\_unmeasured__: value of lambda in the unmeasured cell. 0 by default

#### sensor namespace
>__error\_region\_x__: mismeasurement along the x axis of the range sensor in the beam frame, in meters.

>__error\_region\_y__: mismeasurement along the y axis of the range sensor in the beam frame, in meters.

>__ph__: probability of true positive 'hit'

>__pm__: probability of true positive 'miss'

#### topic namespace
>__laserScan__: topic on which the data sensor are published   

>__occupancyGrid__: topic on which the occupancy grid is published to display the simulation  

>__massGrid__: topic on which the mass grid is subscribed (optional)

>__lambdaGrid__: topic on which the lambda grid is published

>__publish\_rate__: publication rate of ros topics  

#### tf namespace
>__odom__: the world-fixed frame  

>__lidar__: the sensor frame (here a lidar)  

>__robot__: the robot frame  

#### nomals\_estimator namespace
>__neighbor\_radius__: maximum number of points used to estimate the normal of each point

>__max\_radius__: maximum distance in meter between two points

>__min\_neighbors__: minimum number of neighbors to estimate the normal.

## 4.3 Required tf transforms
laser -> base\_link
>usually a fixed value, broadcast periodically by a static\_transform\_publisher

base\_link -> odom
>usually provided by the odometry system
