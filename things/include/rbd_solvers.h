#ifndef _RBD_SOLVERS_H
#define _RBD_SOLVERS_H

#include "GISolver.h"
#include "rigid_body.h"
#include "torque.h"
#include "the_wheel.h"
#include "collision_handler.h"

namespace pba{

class AdvanceRotationAndCOM : public GISolverBase {
public:
	AdvanceRotationAndCOM(RB_sp rbd) : _rbd(rbd) {}

	void init() override {};
	virtual void solve(const double dt) override;


protected:
	AdvanceRotationAndCOM() = delete;
	RB_sp _rbd;

};

class AdvanceRotationAndCOMWithCollisions : public AdvanceRotationAndCOM {
public:
	AdvanceRotationAndCOMWithCollisions(RB_sp);
	void solve(const double dt) override;
	void solve_no_collisions_and_populate_pos_and_updated_pos(const double dt);

private:
	RBDCollisionHandler rbd_coll_handler;
};

class AdvanceAngularVelocityAndVelocity : public GISolverBase {
public:
	AdvanceAngularVelocityAndVelocity( RB_sp rbd, Force_sp f) : _rbd(rbd), _torque(Torque(f)) {}; 
	~AdvanceAngularVelocityAndVelocity() = default;

	void init() override {};
	void solve(const double dt) override;

private:
	RB_sp _rbd;
	Torque _torque;
};



} // end namespace pba


#endif