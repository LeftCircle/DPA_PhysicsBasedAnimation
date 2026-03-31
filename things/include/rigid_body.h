#ifndef _RIGID_BODY_H
#define _RIGID_BODY_H

#include <vector>
#include <memory>
#include <numeric>

#include "dynamical_state_data.h"


namespace pba {


class RigidBodyStateData : public DynamicalStateData{
public:
	RigidBodyStateData() = default;
	~RigidBodyStateData() = default;

	// We need to override the add/resize methods
	// since we _should_ recompute some things from them

	void compute_com();
	void compute_com_for_loop();

	Vector center_of_mass;

private:

};

using RB_sp = std::shared_ptr<RigidBodyStateData>;



} // end namespace pba



#endif