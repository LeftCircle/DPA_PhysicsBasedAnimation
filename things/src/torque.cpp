#include "torque.h"

using namespace pba;


void Torque::compute(RB_sp& rbd, const double dt){
    _force->compute(rbd, dt);
    // Now determine the angular acceleration
    auto acc = rbd->get_vector_attribute_span("acceleration");
    auto masses = rbd->get_float_attribute_span("mass");
    
    // A simple for loop to fall back on if needed
    // rbd->angular_accel = Vector(0, 0, 0);
    // rbd->com_accel = Vector(0, 0, 0);
    // for (int i = 0; i < rbd->n_particles(); i++){
    //     rbd->angular_accel += (acc[i] / masses[i]) ^ rbd->get_lever_arm(i);
    //     rbd->com_accel += acc[i];
    // }
    // still have to divide by total mass tho
    // TODO -> implement a counting iterator within the_wheel
    
    rbd->angular_accel = std::transform_reduce(
        std::execution::par,
        acc.begin(),
        acc.end(),
        Vector(0, 0, 0),
        [](const Vector&a, const Vector& b) { return a + b; },
        [&, base = acc.data()](const Vector& ai) {
            const size_t i = (size_t)(&ai - base);
            return (ai / masses[i]) ^ rbd->get_rotated_lever_arm(i);
        }
    );
    
    rbd->com_accel = std::accumulate(acc.begin(), acc.end(), Vector(0, 0, 0), 
        [](Vector com_a, const Vector& a){return com_a + a;}
    );
    if (rbd->get_total_mass()){
        rbd->com_accel /= rbd->get_total_mass();
    }
}

// A parrallel reduce that uses onetbb without the 
// need for pointer arithmatic. 
// void Torque::compute(RB_sp& rbd, const double dt){
//     _force->compute(rbd, dt);

//     auto acc = rbd->get_vector_attribute_span("acceleration");
//     auto masses = rbd->get_float_attribute_span("mass");
//     const std::size_t n = rbd->n_particles();

//     rbd->angular_accel = tbb::parallel_reduce(
//         tbb::blocked_range<std::size_t>(0, n),
//         Vector(0.0, 0.0, 0.0),
//         [&](const tbb::blocked_range<std::size_t>& range, Vector local){
//             for (std::size_t i = range.begin(); i != range.end(); ++i){
//                 local += (acc[i] / masses[i]) ^ rbd->get_rotated_lever_arm(i);
//             }
//             return local;
//         },
//         [](const Vector& a, const Vector& b){
//             return a + b;
//         }
//     );
// }