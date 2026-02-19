#include "sph_solver.h"

using namespace pba;


double pba::get_density_with_uniform_h_parallel(
    size_t i,
    const std::span<const size_t>& neighbor_indices,
    const DynamicalStateData_sp& dsd,
    const Kernel& kernel)
{
    const Vector& pos_i = dsd->get_position(i);
	const auto positions = dsd->get_vector_attribute_span("positions");
	const auto masses = dsd->get_float_attribute_span("mass");
	return std::transform_reduce(std::execution::par_unseq, neighbor_indices.begin(), neighbor_indices.end(), 0.0, std::plus<>(), [&pos_i, &positions, &masses, &kernel](size_t j){
		double distance = (pos_i - positions[j]).magnitude();
		return masses[j] * kernel(distance);
	});
}

double pba::get_density_with_uniform_h(
    size_t i,
    const std::span<const size_t>& neighbor_indices,
    const DynamicalStateData_sp& dsd,
    const Kernel& kernel)
{
	const Vector& pos_i = dsd->get_position(i);
	const auto positions = dsd->get_vector_attribute_span("positions");
	const auto masses = dsd->get_float_attribute_span("mass");
	return std::accumulate(neighbor_indices.begin(), neighbor_indices.end(), 0.0, [&pos_i, &positions, &masses, &kernel](double acc, size_t j){
        double distance = (pos_i - positions[j]).magnitude();
        return acc + masses[j] * kernel(distance);
    });
}

double pba::get_density_with_uniform_h_silly_loop(
    size_t i,
    const std::span<const size_t>& neighbor_indices,
    const DynamicalStateData_sp& dsd,
    const Kernel& kernel)
{
	const Vector& pos_i = dsd->get_position(i);
	double density = 0.0;
	for (size_t j : neighbor_indices) {
		const Vector& pos_j = dsd->get_position(j);
		double mass_j = dsd->get_mass(j);
		double distance = (pos_i - pos_j).magnitude();
		density += mass_j * kernel(distance);
	}
	return density;
}