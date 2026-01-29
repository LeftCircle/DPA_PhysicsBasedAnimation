#ifndef _PARTIAL_SOLVERS_H
#define _PARTIAL_SOLVERS_H

#include "GISolver.h"
#include "dynamical_state_data.h"
#include "collision_handler.h"
#include "force.h"

namespace pba{

class PartialSolverAdvancePosition : public GISolverBase
{
public:
	PartialSolverAdvancePosition(DynamicalStateData_sp dsd) : _state_data(dsd) {}
	~PartialSolverAdvancePosition() = default;

	void init() override {}
	void solve(const double dt) override;

private:
	DynamicalStateData_sp _state_data;
};

class AdvancePositionWithCollisions : public GISolverBase{
public:
	AdvancePositionWithCollisions(DynamicalStateData_sp dsd, CollisionHandler_sp collision_handler);
	~AdvancePositionWithCollisions() = default;

	void init() override {}
	void solve(const double dt) override;

private:
	DynamicalStateData_sp _state_data;
	CollisionHandler_sp _collision_handler;
};

inline GISolver_sp create_advance_position_with_collisions(DynamicalStateData_sp dsd, CollisionHandler_sp collision_handler){
	return std::make_shared<AdvancePositionWithCollisions>(dsd, collision_handler);
}

class AdvanceVelocityWithForces : public GISolverBase{
public:
	AdvanceVelocityWithForces(DynamicalStateData_sp dsd, ForceSystem_sp force_system)
	: _state_data(dsd), _force_system(force_system) {}
	~AdvanceVelocityWithForces() = default;

	void init() override {}
	void solve(const double dt) override;

private:
	DynamicalStateData_sp _state_data;
	ForceSystem_sp _force_system;
};


} // end namespace pba

#endif