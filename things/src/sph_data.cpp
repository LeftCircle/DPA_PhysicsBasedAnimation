#include "sph_data.h"

using namespace pba;


SPHData::SPHData() : DynamicalStateData() {
	set_uniform("h", (double)1.0);
	set_uniform("rest_density", (double)1000.0);
	set_uniform("p_bar", (double)200.0);
	set_uniform("gamma", (double)7.0);
	set_uniform("viscocity_alpha", (double)0.1);
	set_uniform("viscocity_beta", (double)0.1);
	set_uniform("viscocity_epsilon", (double)0.01);
}