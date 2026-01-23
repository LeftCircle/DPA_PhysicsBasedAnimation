#include "dynamical_state_data.h"

using namespace pba;

DynamicalStateData::DynamicalStateData() {
	_initialize_default_attributes();
}

void DynamicalStateData::_initialize_default_attributes() {
	// Create and cache default attributes
	_vec_attr["positions"] = DSAv("positions", Vector(0.0f, 0.0f, 0.0f));
	_pos_iter = _vec_attr.find("positions");

	_vec_attr["velocities"] = DSAv("velocities", Vector(0.0f, 0.0f, 0.0f));
	_vel_iter = _vec_attr.find("velocities");

	_vec_attr["accelleration"] = DSAv("accelleration", Vector(0.0f, 0.0f, 0.0f));
	_acc_iter = _vec_attr.find("accelleration");

	_float_attr["mass"] = DSAf("mass", 1.0f);
	_mass_iter = _float_attr.find("mass");

	_color_attr["color"] = DSAc("color", Color(1.0f, 1.0f, 1.0f, 1.0f));
	_color_iter = _color_attr.find("color");
}


size_t DynamicalStateData::add(){
	_n_particles += 1;
	// Expand all attributes to accommodate new particle
	for( auto& [name, attr] : _int_attr ){
		attr.expand_to(_n_particles);
	}
	for( auto& [name, attr] : _float_attr ){
		attr.expand_to(_n_particles);
	}
	for( auto& [name, attr] : _vec_attr ){
		attr.expand_to(_n_particles);
	}
	for( auto& [name, attr] : _color_attr ){
		attr.expand_to(_n_particles);
	}
	return _n_particles;
}





