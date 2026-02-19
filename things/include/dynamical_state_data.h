#ifndef _DYNAMICAL_STATE_DATA_H
#define _DYNAMICAL_STATE_DATA_H

#include <map>
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <span>
#include <variant>

#include "Vector.h"
#include "Color.h"
#include "dynamical_state_attribute.h"

namespace pba{

using UniformValue = std::variant<int, float, double, Vector, Color>;

class DynamicalStateData {
public:
    // Q: Can we still perform move operations since we are defining
    // a non default constructor??
    DynamicalStateData();
    virtual ~DynamicalStateData() = default;
	
	size_t add();
	size_t add(size_t n);

	void resize(size_t n);

	// accessors for attributes by name
	const Vector& get_vector_attribute(const std::string& name, size_t i) const;
	const float& get_float_attribute(const std::string& name, size_t i) const;
	const double& get_double_attribute(const std::string& name, size_t i) const;
	const int& get_int_attribute(const std::string& name, size_t i) const;
	const Color& get_color_attribute(const std::string& name, size_t i) const;

	// accessors for common attributes
	const Vector& get_position(size_t i) const { return _pos_map_iter->second.get(i); }
	const Vector& get_velocity(size_t i) const { return _vel_map_iter->second.get(i); }
	const Vector& get_acceleration(size_t i) const { return _acc_map_iter->second.get(i); }
	const float& get_mass(size_t i) const { return _mass_map_iter->second.get(i); }
	const Color& get_color(size_t i) const { return _color_map_iter->second.get(i); }

	// Span for all attributes
	std::span<const Vector> get_vector_attribute_span(const std::string& name) const;
	std::span<const float> get_float_attribute_span(const std::string& name) const;
	std::span<const double> get_double_attribute_span(const std::string& name) const;
	std::span<const int> get_int_attribute_span(const std::string& name) const;
	std::span<const Color> get_color_attribute_span(const std::string& name) const;

	std::span<Vector> get_vector_attribute_span(const std::string& name);
	std::span<float> get_float_attribute_span(const std::string& name);
	std::span<double> get_double_attribute_span(const std::string& name);
	std::span<int> get_int_attribute_span(const std::string& name);
	std::span<Color> get_color_attribute_span(const std::string& name);

	// setters for attributes by name
	void set_vector_attribute(const std::string& name, size_t i, const Vector& v);
	void set_float_attribute(const std::string& name, size_t i, const float& f);
	void set_double_attribute(const std::string& name, size_t i, const double& d);
	void set_int_attribute(const std::string& name, size_t i, const int& val);
	void set_color_attribute(const std::string& name, size_t i, const Color& c);

	void set_uniform(const std::string& name, UniformValue val) { _uniforms[name] = val; }

	template <typename T>
	const T& get_uniform(const std::string& name) const {
		return std::get<T>(_uniforms.at(name));
	}

	// setters for common attributes
	void set_position(size_t i, const Vector& v) { _pos_map_iter->second.set(i, v); }
	void set_velocity(size_t i, const Vector& v) { _vel_map_iter->second.set(i, v); }
	void set_acceleration(size_t i, const Vector& v) { _acc_map_iter->second.set(i, v); }
	void set_mass(size_t i, const float& m) { _mass_map_iter->second.set(i, m); }
	void set_color(size_t i, const Color& c) { _color_map_iter->second.set(i, c); }


	size_t n_particles() const { return _n_particles; }

	
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
		} else if constexpr (std::is_same_v<T, double>) {
			_double_attr.insert_or_assign(name, std::move(attr));
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
	bool has_double_attribute(const std::string& name) const {
		return _double_attr.find(name) !=  _double_attr.end();
	}

private:
	size_t _n_particles = 0;
	std::map< std::string, DSAi > _int_attr;
	std::map< std::string, DSAf > _float_attr;
	std::map< std::string, DSAv > _vec_attr;
	std::map< std::string, DSAc > _color_attr;
	std::map< std::string, DSAd > _double_attr;
	std::map<std::string, UniformValue> _uniforms;
	

	// caching these to avoid map lookup for commonly used lookups
	std::map< std::string, DSAv >::iterator _pos_map_iter;
	std::map < std::string, DSAv >::iterator _vel_map_iter;
	std::map < std::string, DSAv >::iterator _acc_map_iter;
	std::map < std::string, DSAf >::iterator _mass_map_iter;
	std::map < std::string, DSAc >::iterator _color_map_iter;
	std::string _name;

	void _initialize_default_attributes();
	
	template<typename MapType>
	static auto& _lookup_or_throw(MapType& m, const std::string& name, const std::string& type){
		if (auto it = m.find(name); it != m.end()) return it->second;
		else throw std::runtime_error("No " + type + " attribute with name: " + name);
	}

	void _resize_all_attributes(size_t n);

};

using DynamicalStateData_sp = std::shared_ptr<DynamicalStateData>;

inline DynamicalStateData_sp create_dynamical_state_data() { return std::make_shared<DynamicalStateData>(); };

}; // end namespace pba

#endif