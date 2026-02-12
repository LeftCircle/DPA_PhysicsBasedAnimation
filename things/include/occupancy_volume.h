
#ifndef _OCCUPANCY_VOLUME_H
#define _OCCUPANCY_VOLUME_H

#include <vector>
#include <span>
#include <type_traits>
#include <concepts>

#include "array3D.h"
#include "Vector.h"

#include "AABB.h"

namespace pba {

template <typename CellType>
class OccupancyVolume {
public:
	OccupancyVolume(const AABB& aabb, double cell_size) :
	_aabb(aabb),
	_cell_size(cell_size),
	_voxels(
        (int)(std::ceil((aabb.upper_right() - aabb.lower_left()).X() / cell_size)),
        (int)(std::ceil((aabb.upper_right() - aabb.lower_left()).Y() / cell_size)),
        (int)(std::ceil((aabb.upper_right() - aabb.lower_left()).Z() / cell_size))
	 ) {}

	Vector get_dimensions() const noexcept {
		return Vector(_voxels.get_x_dim(), _voxels.get_y_dim(), _voxels.get_z_dim());
	}

	const CellType& get_cell(int x_idx, int y_idx, int z_idx) const {
		return _voxels(x_idx, y_idx, z_idx);
	}

	template <typename Inserter> 
		requires std::invocable<Inserter, CellType&, size_t>
	void populate(const std::vector<Vector>& positions, Inserter inserter_fn) {
		_voxels.clear();
		for (size_t i = 0; i < positions.size(); i++) {
			CellType& cell = _get_cell(positions[i]);
			inserter_fn(cell, i);
		}
	}

private:
	CellType& _get_cell(const Vector& position) {
		int x_idx = (int)((position.X() - _aabb.lower_left().X()) / _cell_size);
		int y_idx = (int)((position.Y() - _aabb.lower_left().Y()) / _cell_size);
		int z_idx = (int)((position.Z() - _aabb.lower_left().Z()) / _cell_size);
		return _voxels(x_idx, y_idx, z_idx);
	}

	AABB _aabb;
	double _cell_size;
	Array3D<CellType> _voxels;

};

} // end namespace pba


#endif