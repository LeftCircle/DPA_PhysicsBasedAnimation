#ifndef _RBD_SOLVERS_H
#define _RBD_SOLVERS_H

#include "GISolver.h"
#include "rigid_body.h"
#include "torque.h"

namespace pba{

class AdvanceRotationAndCOM : public GISolverBase {
public:
	AdvanceRotationAndCOM(RB_sp rbd) : _rbd(rbd) {}

	void init() override {};
	void solve(const double dt) override;


private:
	RB_sp _rbd;

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