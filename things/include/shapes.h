#ifndef _SHAPES_H
#define _SHAPES_H

#include "Vector.h"

namespace pba{

struct Triangle{
	Triangle(const Vector& a, const Vector& b, const Vector& c)
	: v0(a), v1(b), v2(c) {}
	Vector v0;
	Vector v1;
	Vector v2;
};


} // end namespace pba


#endif