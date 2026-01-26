#ifndef _PARTIAL_SOLVERS_H
#define _PARTIAL_SOLVERS_H

#include "GISolver.h"
#include "dynamical_state_data.h"
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


} // end namespace pba

#endif