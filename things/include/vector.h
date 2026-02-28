#ifndef _CATO_VECTOR_H
#define _CATO_VECTOR_H

namespace cato{

template<typename T>
class Vector3 {
public:
	Vector3() : _x(static_cast<T>(0)), _y(static_cast<T>(0)), _z(static_cast<T>(0)) {};
	Vector3(T x, T y, T z) : _x(static_cast<T>(x)), _y(static_cast<T>(y)), _z(static_cast<T>(z)) {};

private:
	T _x;
	T _y;
	T _z;
};

using Vec3i = Vector3<int>;
}


#endif