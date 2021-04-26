#include "include/lambdaCell.hpp"

float LambdaCell::error_region_area = 1; //m^2
float LambdaCell::pm = 1; //.
float LambdaCell::ph = 1; //.
float LambdaCell::mass_array_step = 1; //kg

LambdaCell::LambdaCell(): 
	hit(0), miss(0),
	C_normal(0), S_normal(0)
{
	for(int i=0; i<MASS_ARRAY_SIZE-1; ++i)
		pdf_mass_kg[i] = 0.f;
	pdf_mass_kg[MASS_ARRAY_SIZE-1]=1.0; //P(m=inf)=1
}

void LambdaCell::inc_hit(){
	++hit;
	update();
}
void LambdaCell::inc_miss(){
	++miss;
	update();
}
void LambdaCell::update(){
	if (hit*hit + miss*miss > RADIUS_HIT_MISS*RADIUS_HIT_MISS)
	{
		hit  *= RADIUS_HIT_MISS/std::sqrt(hit*hit+miss*miss);
		miss *= RADIUS_HIT_MISS/std::sqrt(hit*hit+miss*miss);
	}
}

void LambdaCell::update_normal(float n){
	C_normal += std::cos(n);
	S_normal += std::sin(n);
}

void LambdaCell::update_mass_pdf(std::array<float, MASS_ARRAY_SIZE> &pdf){ 
	pdf_mass_kg = pdf;
}

float LambdaCell::get_hit() const{
	return hit;
}
float LambdaCell::get_miss() const{
	return miss;
}

float LambdaCell::lambda() const{
	if(hit == 0 && miss==0) 
		return 0;
	else if (miss ==0) 
		return std::numeric_limits<float>::max();
	else
		return 1./error_region_area*std::log(1+(float)hit/(float)miss);
}
float LambdaCell::lambda_up() const{
	float mu = hit*ph + miss*(1-pm);
	float sig2 = hit*ph*(1-ph) + miss*pm*(1-pm);

	float KU = std::min(mu + 1.96 * std::sqrt(sig2), (double)miss+hit);
	if(KU >= miss+hit)
		return std::numeric_limits<float>::max();

	float lU = 1./error_region_area * std::log(KU/(miss+hit-KU)+1.);

	return lU;
}
float LambdaCell::lambda_low() const{
	if(hit == 0 && miss == 0) //useful to check if the cell has been measured
		return 0;

	float mu = hit*ph + miss*(1-pm);
	float sig2 = hit*ph*(1-ph) + miss*pm*(1-pm);

	float KL = std::max(mu - 0.96 * std::sqrt(sig2), 0.);
	if(KL <= 0.)
		return 0.;
	float lL = 1./error_region_area * std::log(KL/(miss+hit-KL)+1.);

	return lL;
}
float LambdaCell::normal() const{
	if(C_normal==0 and S_normal==0)
		return NAN;
	return std::atan2(S_normal, C_normal);
}
const std::array<float, MASS_ARRAY_SIZE>& LambdaCell::pdf_mass() const{
	return pdf_mass_kg;
}
bool LambdaCell::is_measured() const{
	return !(hit==0 && miss==0);
}
