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

	double get_max_particle_speed() const { return _max_particle_speed; }
	double get_max_particle_acceleration() const { return _max_particle_acceleration; }
	void set_max_particle_speed(double speed) { _max_particle_speed = speed; }
	void set_max_particle_acceleration(double accel) { _max_particle_acceleration = accel; }

private:
	double _max_particle_speed = 1000.0;
	double _max_particle_acceleration = 1000.0;
};

using SPHData_sp = std::shared_ptr<SPHData>;

} // end namespace pba


#endif