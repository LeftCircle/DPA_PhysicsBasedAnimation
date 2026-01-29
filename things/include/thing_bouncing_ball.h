#ifndef _THING_BOUNCING_BALL_H
#define _THING_BOUNCING_BALL_H


#include "dynamical_state_data.h"
#include "GISolver.h"
#include "force_library.h"
#include "partial_solvers.h"
#include "PbaThing.h"
#include "collision_handler.h"

namespace pba{

class BouncingBallThing : public PbaThingyDingy{
public:

	BouncingBallThing(const std::string& nam = "BouncingBallThing");
	~BouncingBallThing() = default;

	void Init( const std::vector<std::string>& args ) override;

	void Display() override;

	void Keyboard( unsigned char key, int x, int y ) override;

	void solve() override;

	void Reset() override;

	void Usage() override;

	std::map<std::string,std::string> MetaData() const override;

private:
	DynamicalStateData_sp _dsd;
	ForceSystem_sp _force_system;
	std::shared_ptr<GISolverSystem> _solver_system;
	CollisionHandler_sp _collision_handler;
	CollisionSurface_sp _box;

	void _initialize_box_collision_surface();
	void _draw_box();

	BouncingBallThing() = delete;
};


} // end namespace pba




#endif








