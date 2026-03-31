#ifndef _RBD_SOLVERS_H
#define _RBD_SOLVERS_H

#include "GISolver.h"
#include "rigid_body.h"

namespace pba{

class AdvanceRotationAndCOM : public GISolverBase {
public:
	AdvanceRotationAndCOM(RB_sp rbd) : _rbd(rbd) {}

	void init() override {};
	void solve(const double dt) override;


private:
	RB_sp _rbd;

};



} // end namespace pba


#endif