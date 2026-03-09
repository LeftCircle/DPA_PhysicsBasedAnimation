#ifndef _SOFT_EDGE_H
#define _SOFT_EDGE_H

#include <span>
#include "Vector.h"

namespace pba{

class SoftEdge{
public:
	using idxs = std::pair<size_t, size_t>;	

	SoftEdge(size_t i, size_t j, double rest_len) : _index_a(i), _index_b(j), _rest_length(rest_len) {}

	// Might be cleaner to pass in the DSD since we have indices already, but that's a lot of shared pointer passing
	void compute(std::span<const Vector> positions, std::span<const Vector> vels, const double spring_force, const double friction);
	const Vector& get_force_on_a() const noexcept { return _force_on_a; }
	idxs get_indices() const noexcept {return idxs(_index_a, _index_b); }

private:
	SoftEdge() = delete;

	size_t _index_a, _index_b;
	double _rest_length;
	Vector _force_on_a = Vector(0.0, 0.0, 0.0); 
};

class SoftTriangle{

};


}// end namepspace pba




#endif