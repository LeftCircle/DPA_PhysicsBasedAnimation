#ifndef _PARTICLE_RBD_COLLISIONS_H
#define _PARTICLE_RBD_COLLISIONS_H

#include "rbd_solvers.h"
#include "shapes.h"

using namespace pba;

struct ParticleRBDHitResult {
	double time;
	size_t particle;
	Vector rbd_indices;
	Vector position;
	Vector normal;
	bool converged = false;
};

class ParticleRBDCollisionHandler : public RBDCollisionHandler{
public:
	virtual void register_rbd(RB_sp rbd) { _rbds.push_back(rbd); }

	void handle_collisions(
		DynamicalStateDataBase_sp dsd,
		const std::string& updated_pos_attr_name,
		const double dt
	) override;

	bool does_moving_particle_collide_with_moving_rbd_tri(
		Vector p_start,
		Vector p_end,
		Triangle tri_start,
		Triangle tri_end
	) const;

private:
	RBDHitResult _find_earliest_rbd_static_collision_from_all_rbds(const double dt) const;
	ParticleRBDHitResult _find_earliest_particle_rbd_collision(DSDB_sp dsd, const double dt) const;
	ParticleRBDHitResult _bisect_particle_rbd_collision(
		size_t particle_idx,
		Vector p_start,
		Vector p_vel,
		Vector rbd_indices,
		RB_sp rbd,
		double dt
	) const;
	std::vector<RB_sp> _rbds;

};



#endif