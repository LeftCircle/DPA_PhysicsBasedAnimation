#ifndef _DYNAMICAL_STATE_DATA_H
#define _DYNAMICAL_STATE_DATA_H

#include <map>
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

#include "Vector.h"
#include "Color.h"
#include "dynamical_state_attribute.h"

namespace pba{

class DynamicalStateData {
public:
    // Q: Can we still perform move operations since we are defining
    // a non default constructor??
    DynamicalStateData();
    ~DynamicalStateData() = default;
	
	size_t add();

    DSAv& get_positions() const noexcept { return _pos_iter->second; }
	DSAv& get_velocities() const noexcept { return _vel_iter->second; }
	DSAv& get_accelleration() const noexcept { return _acc_iter->second; }
	DSAf& get_mass() const noexcept { return _mass_iter->second; }
	DSAc& get_color() const noexcept { return _color_iter->second; }

	template<typename T>
	DSAttribute<T>& get_attribute(const std::string& name){
		if constexpr (std::is_same_v<T, int>) {
			return _lookup_or_throw(_int_attr, name, "int");
		} else if constexpr (std::is_same_v<T, float>) {
			return _lookup_or_throw(_float_attr, name, "float");
		} else if constexpr (std::is_same_v<T, Vector>) {
			return _lookup_or_throw(_vec_attr, name, "vector");
		} else if constexpr (std::is_same_v<T, Color>) {
			return _lookup_or_throw(_color_attr, name, "color");
		} else {
			throw std::runtime_error("Unsupported attribute type requested");
		}
	}
	
	template<typename T>
	void add_attribute(const std::string& name, DSAttribute<T>&& attr){
		attr.expand_to(_n_particles);
		if constexpr (std::is_same_v<T, int>) {
			_int_attr.insert_or_assign(name, std::move(attr));
		} else if constexpr (std::is_same_v<T, float>) {
			_float_attr.insert_or_assign(name, std::move(attr));
		} else if constexpr (std::is_same_v<T, Vector>) {
			_vec_attr.insert_or_assign(name, std::move(attr));
		} else if constexpr (std::is_same_v<T, Color>) {
			_color_attr.insert_or_assign(name, std::move(attr));
		} else {
			throw std::runtime_error("Unsupported attribute type added");
		}
	}

	template<typename T>
	void add_attribute(const std::string& name, const DSAttribute<T>& attr){
		DSAttribute<T> new_attr = attr;
		add_attribute<T>(name, std::move(new_attr));
	}

	bool has_int_attribute(const std::string& name) const{
		return _int_attr.find(name) !=  _int_attr.end();
	}
	bool has_float_attribute(const std::string& name) const {
		return _float_attr.find(name) !=  _float_attr.end();
	}
	bool has_vector_attribute(const std::string& name) const {
		return _vec_attr.find(name) !=  _vec_attr.end();
	}
	bool has_color_attribute(const std::string& name) const {
		return _color_attr.find(name) !=  _color_attr.end();
	}

private:
	size_t _n_particles = 0;
	std::map< std::string, DSAi > _int_attr;
	std::map< std::string, DSAf > _float_attr;
	std::map< std::string, DSAv > _vec_attr;
	std::map< std::string, DSAc > _color_attr;

	// caching these to avoid map lookup for commonly used lookups
	std::map< std::string, DSAv >::iterator _pos_iter;
	std::map < std::string, DSAv >::iterator _vel_iter;
	std::map < std::string, DSAv >::iterator _acc_iter;
	std::map < std::string, DSAf >::iterator _mass_iter;
	std::map < std::string, DSAc >::iterator _color_iter;
	std::string _name;

	void _initialize_default_attributes();
	
	template<typename MapType>
	static auto& _lookup_or_throw(MapType& m, const std::string& name, const std::string& type){
		if (auto it = m.find(name); it != m.end()) return it->second;
		else throw std::runtime_error("No " + type + " attribute with name: " + name);
	}

};

using DynamicalStateData_sp = std::shared_ptr<DynamicalStateData>;

inline DynamicalStateData_sp create_dynamical_state_data() { return std::make_shared<DynamicalStateData>(); };

}; // end namespace pba

#endif