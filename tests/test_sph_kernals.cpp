#include "catch_helpers.h"

#include "sph_kernal.h"
#include "Vector.h"

using namespace pba;

TEST_CASE("Cubic Spline returns values"){
    CubicSplineKernal3 kernal(1.0);
    double val = kernal(0.5);
    Vector grad = kernal.gradient(0.5, Vector(1.0, 0.0, 0.0));
    REQUIRE(val > 0.0);
    REQUIRE(grad.X() < 0.0);
}



