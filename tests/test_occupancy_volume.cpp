#include "catch_helpers.h"

#include "AABB.h"
#include "Vector.h"
#include "occupancy_volume.h"

using namespace pba;

TEST_CASE("Test Occupancy Volume initialization") {
    // An occupancy volume is given an AABB and a cell size. 
    // It should then divide the AABB up into a grid of cell size h. 
    // If the AABB dimensions are not divisible by h, then the grid should
    // encompase all of the AABB plus some extra space on the right and top sides.
    Vector llc(0.0, 0.0, 0.0);
    Vector urc(5.0, 9.3, 13.7);
    AABB aabb(llc, urc);
    double h = 0.66;
    OccupancyVolume<std::vector<size_t>> vol(aabb, h);
    Vector dims = vol.get_dimensions();
    REQUIRE(dims.X() * h >= urc.X() - llc.X());
    REQUIRE(dims.Y() * h >= urc.Y() - llc.Y());
    REQUIRE(dims.Z() * h >= urc.Z() - llc.Z());
    REQUIRE((dims.X() - 1) * h < urc.X() - llc.X());
    REQUIRE((dims.Y() - 1) * h < urc.Y() - llc.Y());
    REQUIRE((dims.Z() - 1) * h < urc.Z() - llc.Z());
}


TEST_CASE("Test populate occupancy volume"){
    
}

