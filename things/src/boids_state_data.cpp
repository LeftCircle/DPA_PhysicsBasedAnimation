#include "boids_state_data.h"


using namespace pba;


void BoidStateData::_initialize_default_attributes() {
    DynamicalStateData::_initialize_default_attributes();
    _double_attr["range"] = DSAd("range", 1.0);
    _double_attr["range_amp"] = DSAd("range_amp", 1.0);
    _double_attr["cos_fov_angle"] = DSAd("cos_fov_angle", std::cos(90 * 3.14159 / 180.0));
    _double_attr["cos_fov_plus_amp"] = DSAd("cos_fov_plus_amp", std::cos(160 * 3.14159 / 180.0));
    _double_attr["coll_avoid_str"] = DSAd("coll_avoid_str", 1.0);
    _double_attr["centering_str"] = DSAd("centering_str", 1.0);
    _double_attr["vel_match_str"] = DSAd("vel_match_str", 1.0);
    _double_attr["acc_budget"] = DSAd("acc_budget", 1.0);
    printf("Boids initialized\n");
}

const BoidParams BoidStateData::get_params(const size_t idx) const {
    return BoidParams{
        get_double_attribute("range", idx),
        get_double_attribute("range_amp", idx),
        get_double_attribute("cos_fov_angle", idx),
        get_double_attribute("cos_fov_plus_amp", idx),
        get_double_attribute("coll_avoid_str", idx),
        get_double_attribute("centering_str", idx),
        get_double_attribute("vel_match_str", idx),
        get_double_attribute("acc_budget", idx)
    };
}