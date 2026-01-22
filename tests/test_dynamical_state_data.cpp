#include "catch_amalgamated.hpp"

#include <map>
#include <vector>

#include <"dynamical_state_data.h>
#include "Vector.h"


TEST_CASE( "cache default state data on creation"){
        // Create dynamical state data and ensure that the default data,
        // positions, velocities, acceleration, and masss are cached. 

        // factory func to create a shared pointer to a dynamical state data object. 
        DynamicalStateData_sp dsd = create_dynamical_state_data();

        dsd.add_particle();
        std::vector<pba::Vector> pos_vec = dsd.get_positions();
        std::map<std::string, std::vector<pba::Vector>>::iterator pos_iter = dsd.get_iterator("position");
        REQUIRE(pos_iter->second == pos_vec);
}



