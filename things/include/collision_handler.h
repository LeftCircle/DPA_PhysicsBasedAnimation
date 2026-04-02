#ifndef _COLLISION_HANDLER_H
#define _COLLISION_HANDLER_H


#include <vector>
#include <memory>
#include <string>
#include <iostream>
#include <map>

#include "collision_surface.h"
#include "dynamical_state_data.h"
#include "rigid_body.h"

namespace pba{

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

	std::vector<const CollisionSurface_sp> collision_surfaces;
};

using CollisionHandler_sp =  std::shared_ptr<CollisionHandler> ;

inline CollisionHandler_sp create_collision_handler() { return std::make_shared<CollisionHandler>(); };

struct RBD_CollisionSurfaceInfo {
	std::vector<int> plane_implicit_start;
	std::vector<int> plane_implicit_end;
	std::vector<double> collision_time;
	std::vector<size_t> colliding_particle;
};

class RBDCollisionHandler : public CollisionHandler {
public:
	void handle_collisions(
		DynamicalStateDataBase_sp dsd,
		const std::string& updated_pos_attr_name,
		const double dt
	) override;

	void register_collision_surface(const CollisionSurface_sp cs) override;

private:
	void _handle_rbd_collisions(RB_sp rbd);

	std::unordered_map<const CollisionSurface_sp, RBD_CollisionSurfaceInfo> _collision_handle_data;
	
};



} // end namespace pba


#endif