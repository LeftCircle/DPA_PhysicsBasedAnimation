#ifndef _RIGID_BODY_H
#define _RIGID_BODY_H

#include <vector>
#include <memory>
#include <numeric>
#include <execution>
#include <algorithm> // std::transform
#include <numeric> // std::reduce


#include "dynamical_state_data.h"
#include "LinearAlgebra.h"


namespace pba {


class RigidBodyStateData : public DynamicalStateDataBase {
public:
	RigidBodyStateData();
	~RigidBodyStateData() = default;

	// We need to override the add/resize methods
	// since we _should_ recompute some things from them
	size_t add() override;
	size_t add(size_t n) override;
	void resize(size_t n) override;

	void set_initial_position(size_t n, const Vector& pos);
	
	void init_rbd() { compute_com(); compute_lever_arms(); }

	const Vector& get_lever_arm(const size_t p) const { return _lever_arms_iter->second.get(p); }
	
	Vector get_vert_pos(const size_t p) const;

	void compute_moi();

	void compute_com();
	void compute_lever_arms();
	void compute_com_for_loop();

	Vector center_of_mass = Vector(0, 0, 0);
	Matrix angular_rotation;
	Vector linear_velocity;
	Vector angular_velocity;
	Vector com_accel;
	Vector angular_accel;
	Vector angular_momentum;

private:
	void _compute_moi(int i, int j) noexcept;

	std::map< std::string, DSAv >::iterator _lever_arms_iter;
	
	Matrix _moment_of_inertia;
	float _total_mass;
	void _initialize_default_attributes() override;
};

using RB_sp = std::shared_ptr<RigidBodyStateData>;



} // end namespace pba



#endif