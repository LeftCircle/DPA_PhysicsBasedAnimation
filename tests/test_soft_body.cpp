#include "catch_helpers.h"

#include <memory>

#include "dynamical_state_data.h"
#include "soft_body_data.h"
#include "math.h"

using namespace pba;


TEST_CASE("test factorial"){
    REQUIRE(factorial<int>(10) == 3628800);
}

TEST_CASE("check number of soft edges in fully connected soft body"){
    // We need to confirm that we are not double counting connections in 
    // the soft body. So the first particle should be connected to all, the second
    // to all but the first, the third to all but the first two, ...

    auto soft_body = std::make_unique<SoftBody>();
    soft_body->add(10);
    soft_body->connect_all_particles_together();
    REQUIRE(soft_body->get_n_soft_edges() == 9 * 10 / 2);


}