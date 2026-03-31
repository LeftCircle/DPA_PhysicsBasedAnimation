#ifndef _RIGID_BODY_H
#define _RIGID_BODY_H

#include <vector>
#include <memory>
#include <numeric>
#include <execution>
#include <algorithm> // std::transform

#if __cplusplus >= 202002L

#endif

#include "dynamical_state_data.h"
#include "LinearAlgebra.h"


namespace pba {


class RigidBodyStateData : public DynamicalStateData{
public:
	RigidBodyStateData();
	~RigidBodyStateData() = default;

	// We need to override the add/resize methods
	// since we _should_ recompute some things from them
	size_t add() override;
	size_t add(size_t n) override;
	void resize(size_t n) override;
	void set_position(size_t i, const Vector& v) override;
	
	void init_rbd() { compute_com(); compute_lever_arms(); }

	// This seems really sloppy. Rigid Bodies still have a get position
	// function, but that function is a lie. We could update it, but then
	// we would have to also update the getters for the position span.
	Vector get_vert_pos(const size_t p) const;

	void compute_com();
	void compute_lever_arms();
	void compute_com_for_loop();

	Vector center_of_mass;
	Matrix angular_rotation;

private:
	float _total_mass;
};

using RB_sp = std::shared_ptr<RigidBodyStateData>;



} // end namespace pba



#endif