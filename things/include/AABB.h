#ifndef _AABB_H
#define _AABB_H

#include "Vector.h"

namespace pba{

class AABB {

public:
	AABB(const Vector& lower_left, const Vector& upper_right)
	: llc(lower_left), urc(upper_right) {}

	const Vector& lower_left() const { return llc; }
	const Vector& upper_right() const { return urc; }
	bool contains(const Vector& point) const {
		return (point.X() >= llc.X() && point.X() <= urc.X() &&
				point.Y() >= llc.Y() && point.Y() <= urc.Y() &&
				point.Z() >= llc.Z() && point.Z() <= urc.Z());
	}

private:
	Vector llc;
	Vector urc;
};

} // end namespace pba
#endif