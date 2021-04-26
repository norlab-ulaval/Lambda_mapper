#pragma once
#include <array>
#include <limits>
#include <cmath>

#define MASS_ARRAY_SIZE 20
#define RADIUS_HIT_MISS 100.f // cap the maximum norm of the vector (hit, miss) for overflows and better recovery time (cf IJRR article)


class LambdaCell
{
  public:
	LambdaCell();

	void inc_hit();
	void inc_miss();
	void update();

	void update_normal(float n);

	void update_mass_pdf(std::array<float, MASS_ARRAY_SIZE> &pdf);

	float get_hit() const;
	float get_miss() const;

	float lambda() const;
	float lambda_up() const;
	float lambda_low() const;

	float normal() const;
	const std::array<float, MASS_ARRAY_SIZE>& pdf_mass() const;

	bool is_measured() const;

	static void set_error_region_area(float error_region_area_){
		error_region_area=error_region_area_;
	}
	static void set_ph(float ph_){
		ph=ph_;
	}
	static void set_pm(float pm_){
		pm=pm_;
	}

	static float get_mass_array_step(){
		return mass_array_step;
	}
	static void set_mass_array_step(float mass_array_step_){
		mass_array_step = mass_array_step_;
	}

  private:
	float hit,miss;
	float C_normal;
	float S_normal;
	static float error_region_area;
	static float ph;
	static float pm;

	std::array<float, MASS_ARRAY_SIZE> pdf_mass_kg;
	static float mass_array_step; 
};
