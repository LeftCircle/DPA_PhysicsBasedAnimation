#include "sph_data.h"

using namespace pba;


SPHData::SPHData() : DynamicalStateData() {
	add_attribute<double>("density", DSAd("density", 1.0));
}