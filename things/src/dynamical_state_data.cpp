#include "dynamical_state_data.h"

using namespace pba;


void DynamicalStateData::_initialize_default_attributes() {
	// Create and cache default attributes
	_vec_attr["positions"] = DSAv("positions", Vector(0.0f, 0.0f, 0.0f));
	_pos_map_iter = _vec_attr.find("positions");

	_vec_attr["velocities"] = DSAv("velocities", Vector(0.0f, 0.0f, 0.0f));
	_vel_map_iter = _vec_attr.find("velocities");

	_vec_attr["accelleration"] = DSAv("accelleration", Vector(0.0f, 0.0f, 0.0f));
	_acc_map_iter = _vec_attr.find("accelleration");

	_float_attr["mass"] = DSAf("mass", 1.0f);
	_mass_map_iter = _float_attr.find("mass");

	_color_attr["color"] = DSAc("color", Color(1.0f, 1.0f, 1.0f, 1.0f));
	_color_map_iter = _color_attr.find("color");
}

