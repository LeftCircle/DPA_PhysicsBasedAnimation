#ifndef _COLLISION_HANDLER_H
#define _COLLISION_HANDLER_H


#include <vector>
#include <memory>
#include <string>
#include <iostream>
#include <unordered_map>
#include <optional>

#include "collision_surface.h"
#include "dynamical_state_data.h"
#include "rigid_body.h"

namespace pba{

inline constexpr double BISEC_TOLERANCE = 0.001;
inline constexpr int MAX_BISEC_ITERS = 100;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

struct CollisionHandleInfo {
	CollisionHandleInfo() {}
	CollisionHandleInfo(CollisionSurface_sp cs, CollisionHitInfo hi) : 
		collision_surface(cs), hit_info(hi) {}
	CollisionSurface_sp collision_surface;
	CollisionHitInfo hit_info;
};

struct ParticleUpdateInfo {
	ParticleUpdateInfo(Vector& sp, Vector& up, Vector& v, double r_dt) :
		start_pos(sp), updated_pos(up), velocity(v), remaining_dt(r_dt) {}
	Vector& start_pos;
	Vector& updated_pos;
	Vector& velocity;
	double remaining_dt;
};

class CollisionHandler{
public:
	CollisionHandler() = default;
	~CollisionHandler() = default;

	virtual void register_collision_surface(const CollisionSurface_sp cs) { collision_surfaces.push_back(cs); }
	virtual void handle_collisions(DynamicalStateDataBase_sp dsd, const std::string& updated_pos_attr_name, const double dt);
	
protected:
	virtual void _add_required_attributes(DynamicalStateDataBase_sp dsd) const {}
	void _handle_particle_collisions(Vector& start_pos, Vector& updated_pos, Vector& velocity, const double dt) const;
	bool _check_for_collision_against_all_surfaces(CollisionHandleInfo& earliest_hit, CollisionHitInfo& temp_hit, ParticleUpdateInfo& pui) const;
	void _on_collision_detected(CollisionHandleInfo& earliest_hit, ParticleUpdateInfo& pui) const noexcept;
	Vector _resolve_collision_against_static_object(
		ParticleUpdateInfo& pui,
		const Vector& collision_position,
		const Vector& hit_normal,
		const double restitution, 
		const double sticky
	) const noexcept;

	std::vector<CollisionSurface_sp> collision_surfaces;
};

using CollisionHandler_sp =  std::shared_ptr<CollisionHandler> ;

inline CollisionHandler_sp create_collision_handler() { return std::make_shared<CollisionHandler>(); };

// TO DO -> we are probably getting rid of this entirely
struct RBD_CollisionSurfaceInfo {
	std::vector<double> plane_implicit_start;
	std::vector<double> plane_implicit_end;
	std::vector<double> collision_time;
	std::vector<size_t> colliding_particle;
	std::vector<Vector> hit_pos;
	std::vector<Vector> hit_normal;
};

struct RBDHitResult {
	double time;
	size_t particle;
	Vector position;
	Vector normal;
	CollisionSurface_sp surface;
};

class RBDCollisionHandler : public CollisionHandler {
public:
	RBDCollisionHandler() = default;
	~RBDCollisionHandler() = default;

	void handle_collisions(
		DynamicalStateDataBase_sp dsd,
		const std::string& updated_pos_attr_name,
		const double dt
	) override;

	void register_collision_surface(const CollisionSurface_sp cs) override;

private:
	void _handle_rbd_collisions(RB_sp rbd, const double dt);

	std::unordered_map<CollisionSurface_sp, RBD_CollisionSurfaceInfo> _collision_handle_data;
	
};

// func is the update function for this specific type of dsd that will 
// update ONLY the particle we are interested in. 
// Should have signature (const DSD_sp& dsd, const size_t particle_idx, const double dt);
template<typename func>
std::optional<RBDHitResult> bisect_collision(
	const DSD_sp& dsd, size_t particle_idx,
	const CollisionObject_sp& cobj, const CollisionSurface_sp& cs,
	double max_t, func pos_update)
{
	const Vector& n = cobj->get_normal();
	double f_start = n * dsd->get_position(particle_idx);
	// predict x_end for JUST this one particle
	Vector x_end = pos_update(dsd, particle_idx, max_t);
	double f_end = n * x_end;

	if (f_start * f_end > 0){
		return std::nullopt;
	}
	// We have a collision! Time to bisect
	double t0 = 0, t1 = max_t, f1 = f_start, th;
	Vector x_mid;
	for (int step = 0; step <= MAX_BISEC_ITERS; step++){
		th = (t0 + t1) / 2.0;
		// Do we have to swap the direction of the update each time?
		x_mid = pos_update(dsd, particle_idx, th);
		double fmid = x_mid * n;
		if (std::abs(fmid) < BISEC_TOLERANCE){
			Vector norm = (rbd->center_of_mass - x_mid) * n > 0 ? n : -n;
			return RBDHitResult(th, particle_idx, x_mid, norm, cs);
		}
		if (f1 * fmid > 0){
			t0 = th;
		} else {
			t1 = th;
		}
	}
	// did not converge. Just return what we have
	Vector norm = (rbd->center_of_mass - x_mid) * n > 0 ? n : -n;
	return RBDHitResult(th, particle_idx, x_mid, norm, cs);
}



} // end namespace pba


#endif