#ifndef _SOFT_BODY_DATA_H
#define _SOFT_BODY_DATA_H


#include <vector>
#include <memory>
#include <algorithm>

#include "dynamical_state_data.h"
#include "soft_edge.h"
#include "math.h"


namespace pba{


class SoftBody : public DynamicalStateData {
public:
	void connect_all_particles_together() noexcept;
	constexpr void clear_connections() noexcept {_edges.clear(); }

	constexpr size_t get_n_soft_edges() const noexcept { return _edges.size(); }

private:
	std::vector<SoftEdge> _edges;
};



} // end namespace pba



#endif