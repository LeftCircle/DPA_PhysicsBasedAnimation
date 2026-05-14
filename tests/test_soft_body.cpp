#include "catch_helpers.h"

#include <memory>
#include <ranges>

#include "dynamical_state_data.h"
#include "soft_body_data.h"
#include "math.h"
#include "shapes.h"
#include "force_library.h"
#include "the_wheel.h"

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

TEST_CASE("test no net force in soft triangles"){
    // Create a dynamical state data with three particles
    // Create a soft triangle based on those particles
    // calculate the soft triangle force on those particles
    // ensure that the sum of the three forces is zero

    // auto dsd = std::make_shared<SoftBody>();
    // dsd->add(3);
    // dsd->set_position(0, Vector(0, 0, 0));
    // dsd->set_position(1, Vector(1, 0, 0));
    // dsd->set_position(2, Vector(0, 1, 0));

    // double area = Triangle::get_area(dsd->p(0), dsd->p(1), dsd->p(2));
    // SoftTriangle soft_tri(0, 1, 2, area, 1.0);
    // std::vector<SoftTriangle> soft_triangles{soft_tri};
    // span<const SoftTriangle> soft_tris_span(soft_triangles);
    // SoftTriangleForce stf(tri_provider);
  
    // stf.compute(dsd, 1.0);

    // // Require that forces on all are zero
    // REQUIRE(dsd->a(0).X() == 0.0 && dsd->a(0).Y() == 0.0 && dsd->a(0).Z() == 0.0);

    // // Now perturb the triangle and check forces
    // dsd->set_position(0, Vector(-1, -1, 0));
    

}

TEST_CASE("test soft triangle force can be used with Accumulating Force"){
    REQUIRE(false);
}


TEST_CASE("Test piping"){
    auto dsd = std::make_shared<SoftBody>();
    dsd->add(3);
    dsd->set_position(0, Vector(0, 0, 0));
    dsd->set_position(1, Vector(1, 0, 0));
    dsd->set_position(2, Vector(0, 1, 0));

    double area = Triangle::get_area(dsd->p(0), dsd->p(1), dsd->p(2));
    SoftTriangle soft_tri(0, 1, 2, area, 1.0);
    SoftTriangle soft_tri_2(1, 2, 0, area, 1.0);
    SoftTriangle soft_tri_3(2, 0, 1, area, 1.0);
    std::vector<SoftTriangle> soft_triangles{soft_tri, soft_tri_2, soft_tri_3};

    auto get_area = [&dsd](const SoftTriangle& st){ 
        double area= Triangle::get_area(dsd->p(st.idx0()), dsd->p(st.idx1()), dsd->p(st.idx2())); 
        return std::pair(&st, area);
    };
    auto updated_tris = soft_triangles | std::views::transform(get_area);
    std::vector<std::pair<const SoftTriangle*, double>> areas_vec;
    
    auto get_edges = [&dsd](const SoftTriangle& st){
        std::array<Vector, 3> edges;
        edges[0] = dsd->p(1) - dsd->p(0);
        edges[1] = dsd->p(2) - dsd->p(0);
        edges[2] = dsd->p(2) - dsd->p(1);
        return edges;
    };

    auto edges = soft_triangles | std::views::transform(get_edges);

    // for (auto a : areas){
    //     areas_vec.push_back(a);
    // }
    // REQUIRE(areas_vec.size() == 3);

    // Let's calculate the force and see that it is zero
    // auto forces = soft_triangles | std::views::transform()
    // 1. determine areas
	// 2. determine edges/midpoints
	// 3. get directions
	// 4. compute force

}