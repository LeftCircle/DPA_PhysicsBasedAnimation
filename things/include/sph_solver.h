#ifndef _SPH_SOLVER_H
#define _SPH_SOLVER_H

#include <numeric>
#include <span>
#include <execution>

#include "partial_solvers.h"
#include "sph_kernel.h"

namespace pba{

// Returns the SPH density for a particle assuming uniform h of 
// all particles in the kernel
double get_density_with_uniform_h_parallel(
	size_t i,
	const std::span<const size_t>& neighbor_indices,
	const DynamicalStateData_sp& dsd,
	const Kernel& kernel
);
double get_density_with_uniform_h(
	size_t i,
	const std::span<const size_t>& neighbor_indices,
	const DynamicalStateData_sp& dsd,
	const Kernel& kernel
);
double get_density_with_uniform_h_silly_loop(
	size_t i,
	const std::span<const size_t>& neighbor_indices,
	const DynamicalStateData_sp& dsd,
	const Kernel& kernel
);



} // end namespace pba





#endif