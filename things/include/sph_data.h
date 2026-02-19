#ifndef _SPH_DATA_H
#define _SPH_DATA_H


#include "dynamical_state_data.h"

// The SPHData class adds the required data components to DSD
// that are used in SPH solvers and forces
namespace pba {
class SPHData : public DynamicalStateData {
public:
	SPHData();
	~SPHData() = default;
};

using SPHData_sp = std::shared_ptr<SPHData>;

} // end namespace pba


#endif