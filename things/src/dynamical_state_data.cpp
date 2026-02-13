#include "dynamical_state_data.h"

using namespace pba;

DynamicalStateData::DynamicalStateData() {
	_initialize_default_attributes();
}

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


size_t DynamicalStateData::add(){
	_n_particles += 1;
	_resize_all_attributes(_n_particles);
	return _n_particles;
}

size_t DynamicalStateData::add(size_t n){
	_n_particles += n;
	_resize_all_attributes(_n_particles);
	return _n_particles;
}

void DynamicalStateData::resize(size_t n){
	_n_particles = n;
	_resize_all_attributes(_n_particles);
}

const Vector& DynamicalStateData::get_vector_attribute(const std::string& name, size_t i) const {
	return _lookup_or_throw(_vec_attr, name, "vector").get(i);
}

const float& DynamicalStateData::get_float_attribute(const std::string& name, size_t i) const {
	return _lookup_or_throw(_float_attr, name, "float").get(i);
}

const double& DynamicalStateData::get_double_attribute(const std::string& name, size_t i) const {
	return _lookup_or_throw(_double_attr, name, "double").get(i);
}

const int& DynamicalStateData::get_int_attribute(const std::string& name, size_t i) const {
	return _lookup_or_throw(_int_attr, name, "int").get(i);
}

const Color& DynamicalStateData::get_color_attribute(const std::string& name, size_t i) const {
	return _lookup_or_throw(_color_attr, name, "color").get(i);
}

void DynamicalStateData::set_vector_attribute(const std::string& name, size_t i, const Vector& v) {
	_lookup_or_throw(_vec_attr, name, "vector").set(i, v);
}

void DynamicalStateData::set_float_attribute(const std::string& name, size_t i, const float& f) {
	_lookup_or_throw(_float_attr, name, "float").set(i, f);
}

void DynamicalStateData::set_double_attribute(const std::string& name, size_t i, const double& d) {
	_lookup_or_throw(_double_attr, name, "double").set(i, d);
}

void DynamicalStateData::set_int_attribute(const std::string& name, size_t i, const int& val) {
	_lookup_or_throw(_int_attr, name, "int").set(i, val);
}

void DynamicalStateData::set_color_attribute(const std::string& name, size_t i, const Color& c) {
	_lookup_or_throw(_color_attr, name, "color").set(i, c);
}

std::span<const Vector> DynamicalStateData::get_vector_attribute_span(const std::string& name) const {
	return _lookup_or_throw(_vec_attr, name, "vector").get_span();
}

std::span<const float> DynamicalStateData::get_float_attribute_span(const std::string& name) const {
	return _lookup_or_throw(_float_attr, name, "float").get_span();
}

std::span<const double> DynamicalStateData::get_double_attribute_span(const std::string& name) const {
	return _lookup_or_throw(_double_attr, name, "double").get_span();
}

std::span<const int> DynamicalStateData::get_int_attribute_span(const std::string& name) const {
	return _lookup_or_throw(_int_attr, name, "int").get_span();
}

std::span<const Color> DynamicalStateData::get_color_attribute_span(const std::string& name) const {
	return _lookup_or_throw(_color_attr, name, "color").get_span();
}

std::span<Vector> DynamicalStateData::get_vector_attribute_span(const std::string& name) {
	return _lookup_or_throw(_vec_attr, name, "vector").get_span();
}

std::span<float> DynamicalStateData::get_float_attribute_span(const std::string& name) {
	return _lookup_or_throw(_float_attr, name, "float").get_span();
}

std::span<double> DynamicalStateData::get_double_attribute_span(const std::string& name) {
	return _lookup_or_throw(_double_attr, name, "double").get_span();
}

std::span<int> DynamicalStateData::get_int_attribute_span(const std::string& name) {
	return _lookup_or_throw(_int_attr, name, "int").get_span();
}

std::span<Color> DynamicalStateData::get_color_attribute_span(const std::string& name) {
	return _lookup_or_throw(_color_attr, name, "color").get_span();
}

void DynamicalStateData::_resize_all_attributes(size_t n){
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
	for( auto& [name, attr] : _double_attr ){
		attr.expand_to(_n_particles);
	}
}





