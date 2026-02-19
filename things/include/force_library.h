#ifndef _FORCE_LIBRARY_H
#define _FORCE_LIBRARY_H

#include <vector>
#include <memory>
#include <span>

#include "force.h"
#include "occupancy_volume.h"
#include "sph_kernel.h"
#include "sph_data.h"

namespace pba{


class SimpleGravityForce : public ForceBase{
public:
	SimpleGravityForce(const Vector& gravity) : _gravity(gravity) {}
	~SimpleGravityForce() = default;

	void compute(DynamicalStateData_sp dsd, const double dt) const override;

	const Vector& get_gravity() const noexcept { return _gravity; }
	void set_gravity(const Vector& gravity) { _gravity = gravity; }

private:
	Vector _gravity;

};

class SPHPressureForce : public ForceBase {
public:
	using idx_volume = std::shared_ptr<OccupancyVolume<std::vector<size_t>>>;
	
	SPHPressureForce(idx_volume occupancy_volume, Kernel_sp kernel) : 
		_occupancy_volume(occupancy_volume), _kernel(kernel) {}
	~SPHPressureForce() = default;

	// This is a bit of a hack to let us use the same force interface for SPH and standard forces
	void compute(DynamicalStateData_sp dsd, const double dt) const override {
		auto sph_data = std::dynamic_pointer_cast<SPHData>(dsd);
		if (!sph_data) throw std::runtime_error("SPHPressureForce requires SPHData");
		compute_sph(sph_data, dt);
	};

	virtual void compute_sph(SPHData_sp sph_data, const double dt) const;

private:
	Vector _get_pressure_with_uniform_h(
		size_t i,
		const std::span<const size_t>& neighbor_indices,
		const SPHData_sp sph_data,
		const std::span<const double>& densities,
		double p_bar,
		double rest_density,
		double gamma
	) const;

	double _get_pressure_at_density(double density, double p_bar, double rest_density, double gamma) const;


	SPHPressureForce() = delete;
	idx_volume _occupancy_volume;
	Kernel_sp _kernel;
};

class SPHViscocityForce : public ForceBase {
public:
	using idx_volume = std::shared_ptr<OccupancyVolume<std::vector<size_t>>>;
	SPHViscocityForce(idx_volume occupancy_volume) : _occupancy_volume(occupancy_volume) {}
	~SPHViscocityForce() = default;

	void compute(DynamicalStateData_sp dsd, const double dt) const override;

private:
	SPHViscocityForce() = delete;
	idx_volume _occupancy_volume;
};

} // end namespace pba


#endif