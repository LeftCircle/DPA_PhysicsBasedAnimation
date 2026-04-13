#ifndef _BOID_STATE_DATA_H
#define _BOID_STATE_DATA_H

#include "dynamical_state_data.h"
#include "boids_acceleration.h"

namespace pba{

class BoidStateData : public DynamicalStateData{
public:
	const BoidParams get_params(const size_t idx) const;

private:
	void _initialize_default_attributes() override;
};

// _double_attr["centering_str"] = DSAd("centering_str", 1.0);
//     _double_attr["vel_match_str"] = DSAd("vel_match_str", 1.0);
//     _double_attr["coll_avoid_str"] = DSAd("coll_avoid_str", 1.0);
//     _double_attr["acc_budget"] = DSAd("acc_budget", 1.0);
//     _double_attr["range"] = DSAd("range", 1.0);
//     _double_attr["range_amp"] = DSAd("range_amp", 1.0);
//     _double_attr["cos_fov_angle"] = DSAd("cos_fov_angle", std::cos(120));
//     _double_attr["cos_fov_plus_amp"] = DSAd("cos_fov_plus_amp", std::cos(180));


}// end namespace pba


#endif