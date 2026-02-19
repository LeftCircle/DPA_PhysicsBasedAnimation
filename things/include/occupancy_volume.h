
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

struct indices {
	size_t i;
	size_t j;
	size_t k;
};

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

	template <typename T, typename func>
		requires std::invocable<func, size_t, T, const CellType&>
	T accumulate_neighbor_cells(size_t idx, const Vector& position, T init, func fn) const {
		auto neighbor_cells = _get_neighbor_cell_indices(position);
		for (const auto& idx : neighbor_cells) {
			init = fn(idx, std::forward<T>(init), _voxels(idx.i, idx.j, idx.k));
		}
		return init;
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
	indices _get_cell_indices(const Vector& position) const {
		int x_idx = (int)((position.X() - _aabb.lower_left().X()) / _cell_size);
		int y_idx = (int)((position.Y() - _aabb.lower_left().Y()) / _cell_size);
		int z_idx = (int)((position.Z() - _aabb.lower_left().Z()) / _cell_size);
		return indices{(size_t)x_idx, (size_t)y_idx, (size_t)z_idx};
	}

	std::vector<indices> _get_neighbor_cell_indices(const Vector& position) const {
		indices idx = _get_cell_indices(position);
		std::vector<indices> neighbor_indices;
		for (int k = -1; k <= 1; k++) {
			for (int j = -1; j <= 1; j++) {
				for (int i = -1; i <= 1; i++) {
					int neighbor_x_idx = idx.i + i;
					int neighbor_y_idx = idx.j + j;
					int neighbor_z_idx = idx.k + k;
					if (neighbor_x_idx >= 0 && neighbor_x_idx < _voxels.get_x_dim() &&
						neighbor_y_idx >= 0 && neighbor_y_idx < _voxels.get_y_dim() &&
						neighbor_z_idx >= 0 && neighbor_z_idx < _voxels.get_z_dim()) {
						neighbor_indices.push_back(indices{(size_t)neighbor_x_idx, (size_t)neighbor_y_idx, (size_t)neighbor_z_idx});
					}
				}
			}
		}
		return neighbor_indices;
	}

	CellType& _get_cell(const Vector& position) {
		indices idx = _get_cell_indices(position);
		return _voxels(idx.i, idx.j, idx.k);
	}

	AABB _aabb;
	double _cell_size;
	Array3D<CellType> _voxels;

};

} // end namespace pba


#endif