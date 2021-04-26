#Estimated p.d.f of the masses in [0, inf]
#The masses in the map, in row-major order, starting with (0,0).
#The size of the map has to be the exact same shape than the one define in config.yaml (i.e., the shape of the lambda grid).
#Each cell is an array of size MASS_ARRAY_SIZE defined in src/include/LambdaCell.hpp, where the ith number is the probability for the mass i*mass_step
#The last number is the probability P(m=inf).
float32[] masses
float32 mass_step
