#include "catch_amalgamated.hpp"

#include <string>
#include <iostream>

#include "Vector.h"


namespace Catch {

template<>
struct StringMaker<pba::Vector>{
	static std::string convert(const pba::Vector& v){
		char buffer[100];
		std::snprintf(buffer, sizeof(buffer), "Vector(%.5f, %.5f, %.5f)", v.X(), v.Y(), v.Z());
		return std::string(buffer);
	}
};

// bool vectors_match(const pba::Vector& v1, const pba::Vector& v2, const double tol = 1e-5){
// 	bool matches =  ( std::abs(v1.X() - v2.X()) < tol &&
// 			 		std::abs(v1.Y() - v2.Y()) < tol &&
// 			 		std::abs(v1.Z() - v2.Z()) < tol );
// 	if (!matches){
// 		std::cout << "Vectors do not match: " << std::endl;
// 		std::cout << "  v1: " << StringMaker<pba::Vector>::convert(v1) << std::endl;
// 		std::cout << "  v2: " << StringMaker<pba::Vector>::convert(v2) << std::endl;
// 	}
// 	return matches;
// }



} // end namespace Catch