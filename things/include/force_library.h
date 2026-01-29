#ifndef _FORCE_LIBRARY_H
#define _FORCE_LIBRARY_H

#include <vector>
#include <memory>

#include "force.h"

namespace pba{


class SimpleGravityForce : public ForceBase{
public:
	SimpleGravityForce(const Vector& gravity) : _gravity(gravity) {}
	~SimpleGravityForce() = default;

	void compute(DynamicalStateData_sp dsd, const double dt) const override;

private:
	Vector _gravity;

};




} // end namespace pba


#endif