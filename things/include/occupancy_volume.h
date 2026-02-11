
#ifndef _OCCUPANCY_VOLUME_H
#define _OCCUPANCY_VOLUME_H

#include "array3D.h"
#include "Vector.h"

#include "AABB.h"

namespace pba {

template <typename CellType>
class OccupancyVolume {
public:
	OccupancyVolume(const AABB& aabb, double cell_size) :
	_voxels(
        (int)(std::ceil((aabb.upper_right() - aabb.lower_left()).X() / cell_size)),
        (int)(std::ceil((aabb.upper_right() - aabb.lower_left()).Y() / cell_size)),
        (int)(std::ceil((aabb.upper_right() - aabb.lower_left()).Z() / cell_size))
	 ) {}

	Vector get_dimensions() const noexcept {
		return Vector(_voxels.get_x_dim(), _voxels.get_y_dim(), _voxels.get_z_dim());
	}

	

private:
	Array3D<CellType> _voxels;

};

} // end namespace pba


#endif