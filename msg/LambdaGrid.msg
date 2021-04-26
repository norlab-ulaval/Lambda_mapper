# This represents a 2-D lambda-grid map, in which each cell represents the intensity function lambda as well as its confidence interval

Header header 

#MetaData for the map
nav_msgs/MapMetaData info

# The map data, in row-major order, starting with (0,0).
# Lamdas are in [0,inf]. The higher the lambda the higher is the likelihood of occupancy
float32[] lambdaE #expected lambda
float32[] lambdaUp
float32[] lambdaLow

# The estimated normals, using the same format as the lambda map
# each normal is in [-pi, pi] and contains nan if no normal could have been estimated
float32[] normals

#Estimated mass in [0, inf]
#Each cell is an array where the ith number is the probability for the mass i*step
#The last number is the probability P(m=inf).
float32[] masses
float32 mass_step
