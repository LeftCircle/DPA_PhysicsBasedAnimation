#include "thing_soft_bunny.h"


using namespace pba;


SoftBunnyThingyDingy::SoftBunnyThingyDingy(const std::string& nam)
: PbaThingyDingy(nam) {

	// Start with some default bounds
	AABB bounds(Vector(-3.0, -3.0, -3.0), Vector(3.0, 3.0, 3.0));
	AABB emission_bounds(Vector(-2.9, -2.9, -2.9), Vector(2.9,  2.9, 2.9));
	
    // Let's add a single bouncing ball particle
	_dsd = std::make_shared<SoftBody>();
	
	// add a thousand particles to start with
	_create_uniform_soft_body_from_obj("", Vector(0, 30, 0));

    // And now our systems, forces and collision surfaces
	_force_system = std::make_shared<ForceSystem>();
	_solver_system = create_gi_solver_system();
	
	_collision_handler = create_collision_handler();
	_main_collision_surface = _create_collision_geo_from("");
	_collision_handler->register_collision_surface(_main_collision_surface);

	_gravity_force = std::make_shared<SimpleGravityForce>(Vector(0.0, -9.81, 0.0));
	_uniform_strut_force = std::make_shared<UniformStrutForce>(20.0, 0.3);
    _force_system->add_forces(_gravity_force, _uniform_strut_force);
	
    
	// And now that the init is basically done. Let's build the solvers
	SetSimulationTimestep(0.001667);
	_set_to_sixth_order_solver();
}


void SoftBunnyThingyDingy::_set_to_backward_euler_solver(){
	// Clear existing solvers
	_solver_system = create_gi_solver_system();

	auto advance_position_solver = std::make_shared<AdvancePositionWithCollisions>(_dsd, _collision_handler);
    auto advance_velocity_solver = std::make_shared<AdvanceVelocityWithForces>(_dsd, _force_system);
	_solver_system->add_solver(advance_velocity_solver, dt);
	_solver_system->add_solver(advance_position_solver, dt);
	printf("Switched to Backward Euler solver.\n");
}

void SoftBunnyThingyDingy::_set_to_forward_euler_solver(){
	// Clear existing solvers
	_solver_system = create_gi_solver_system();

	auto advance_position_solver = std::make_shared<AdvancePositionWithCollisions>(_dsd, _collision_handler);
    auto advance_velocity_solver = std::make_shared<AdvanceVelocityWithForces>(_dsd, _force_system);
	_solver_system->add_solver(advance_position_solver, dt);
	_solver_system->add_solver(advance_velocity_solver, dt);
	printf("Switched to Forward Euler solver.\n");
}

void SoftBunnyThingyDingy::_set_to_leapfrog_solver(){
	_solver_system = create_gi_solver_system();
	auto advance_position_solver = std::make_shared<AdvancePositionWithCollisions>(_dsd, _collision_handler);
    auto advance_velocity_solver = std::make_shared<AdvanceVelocityWithForces>(_dsd, _force_system);
	_solver_system->add_solver(advance_position_solver, dt / 2.0);
	_solver_system->add_solver(advance_velocity_solver, dt);
	_solver_system->add_solver(advance_position_solver, dt / 2.0);
	printf("Switched to Leapfrog solver.\n");
}

void SoftBunnyThingyDingy::_set_to_sixth_order_solver(){
	_solver_system = create_gi_solver_system();
	auto advance_position_solver = std::make_shared<AdvancePositionWithCollisions>(_dsd, _collision_handler);
    auto advance_velocity_solver = std::make_shared<AdvanceVelocityWithForces>(_dsd, _force_system);
	auto leapfrog_solver = std::make_shared<GISolverLeapfrog>(advance_position_solver, advance_velocity_solver);
	auto sixth_order_solver = std::make_shared<GISolverSixthOrder>(leapfrog_solver);
	_solver_system->add_solver(sixth_order_solver, dt);
	printf("Switched to Sixth Order solver.\n");
}

void SoftBunnyThingyDingy::solve(){
	_solver_system->solve(dt);
}

void SoftBunnyThingyDingy::Display(){
	_draw_tris();
	_draw_particles();
}

void SoftBunnyThingyDingy::Keyboard( unsigned char key, int x, int y ){
	switch( key ){
		case 'a':{
			break;
		}
		case 'A':{
			break;
		}
		case 'b':{
			break;
		}
		case 'B':{
			break;
		}
		case 'c':{
			_adjust_coefficient_of_restitution(-0.05);
			break;
		}
		case 'C':{
			_adjust_coefficient_of_restitution(0.05);
			break;
		}
		case 'd':{
			break;
		}
		case 'D':{
			break;
		}
		case 'e':{
			_emit_particles(1);
			break;
		}
		case 'g':{
			_adjust_gravity(Vector(0.0, -0.1, 0.0));
			break;
		}
		case 'G':{
			_adjust_gravity(Vector(0.0, 0.1, 0.0));
			break;
		}
		case 'l':{
			_set_to_leapfrog_solver();
			break;
		}
		case 'L':{
			_set_to_sixth_order_solver();
			break;
		}
		case 'p':{
			break;
		}
		case 'P':{
			break;
		}
		case 'r':{
			Reset();
			break;
		}
		case 's':{
            _adjust_strut_force(0.9);
			break;
		}
		case 'S':{
            _adjust_strut_force(1.1);
			break;
		}
		case 't':{
			_adjust_timestep(0.9);
			break;
		}
		case 'T':{
			_adjust_timestep(1.1);
			break;
		}
		case 'u':
		case 'U':
			Usage();
			break;
		case 'v': {
            _adjust_strut_friction(0.9);
			break;
		}
		case 'V': {
            _adjust_strut_friction(1.1);
			break;
		}
		default:
			break;
	}
}

void SoftBunnyThingyDingy::_emit_particles(const size_t n){
    _create_uniform_soft_body_from_obj("", Vector(0, 30, 0));
	printf("Emitted %zu new particles. Total particle count is now %zu.\n", n, _dsd->n_particles());
}

void SoftBunnyThingyDingy::_adjust_gravity(const Vector& delta){
	_gravity_force->set_gravity(_gravity_force->get_gravity() + delta);
	printf("New gravity vector is (%f, %f, %f) m/s^2\n",
		_gravity_force->get_gravity().X(),
		_gravity_force->get_gravity().Y(),
		_gravity_force->get_gravity().Z()
	);
}

void SoftBunnyThingyDingy::_adjust_coefficient_of_restitution(const double delta){
	_main_collision_surface->set_restitution( _main_collision_surface->get_restitution() + delta );
	printf("New coefficient of restitution is %f\n", _main_collision_surface->get_restitution());
}

void SoftBunnyThingyDingy::Reset(){
	_dsd->resize(0);
	_emit_particles(1);
	printf("Simulation reset.\n");
}

void SoftBunnyThingyDingy::_adjust_timestep(const double factor){
	dt *= factor;
	SetSimulationTimestep(dt);
	printf("New timestep is %f seconds. RESET SOLVER TO APPLY CHANGES\n", dt);
}


void SoftBunnyThingyDingy::_create_uniform_soft_body_from_obj(const std::string& file_name, const Vector& center){
    printf("Hard coding file for now \n");
    std::filesystem::path current_dir = std::filesystem::path(__FILE__).parent_path();
    std::filesystem::path obj_file_path = current_dir / "../../models/bunny_superlo_scaled.obj";

    // we are just going to read it every time for now
    ObjReader<Vector> r(obj_file_path);
    printf("Reading obj from file %s\n", obj_file_path.string().c_str());
    auto verts = r.get_verts();
    auto faces = r.get_faces();
    const size_t n_starting_particles = _dsd->n_particles();
    const Vector vel = ParticleEmitter::generate_random_bounded_vector(0.5, 10);
    for (size_t i = 0; i < r.get_verts().size(); i++){
        _dsd->add();
        _dsd->set_position(i + n_starting_particles, verts[i] + center);
        _dsd->set_velocity(i + n_starting_particles, vel);
    }
    // And now the connections:
    _dsd->connect_all_particles_in_range(n_starting_particles, _dsd->n_particles());
}

CollisionSurface_sp SoftBunnyThingyDingy::_create_collision_geo_from(const std::string& file_name){
	printf("Hard coding collision geo for now");
	std::filesystem::path current_dir = std::filesystem::path(__FILE__).parent_path();
    std::filesystem::path obj_file_path = current_dir / "../../models/bigsphere.obj";
	 ObjReader<Vector> r(obj_file_path);
    printf("Reading obj from file %s\n", obj_file_path.string().c_str());
    auto verts = r.get_verts();
    auto faces = r.get_faces();

	// We need to create collision tris from the faces/verts
	std::vector<Triangle> tris(faces.size(), Triangle(Vector(0, 0, 0), Vector(0, 0, 0), Vector(0, 0, 0)));
	std::transform(std::execution::par, faces.begin(), faces.end(), tris.begin(),
		 [&verts](const cato::Vec3i& face) {
			return Triangle(
				verts[face.x() - 1],
				verts[face.y() - 1],
				verts[face.z() - 1]
			);
	});

	// Add the tris to the collision surface
	CollisionSurface_sp collision_geo = create_collision_surface();
	for (const auto& tri : tris){
		collision_geo->add_collision_object(std::make_shared<CollisionTriangle>(tri));
		_tris_to_draw.push_back(tri);
	}
	return collision_geo;
}

void SoftBunnyThingyDingy::_adjust_strut_force(const double delta){
    _uniform_strut_force->set_spring_force(_uniform_strut_force->get_spring_force() * delta);
    printf("Strut force is now %d\n", _uniform_strut_force->get_spring_force());
}

void SoftBunnyThingyDingy::_adjust_strut_friction(const double delta){
    _uniform_strut_force->set_friction(_uniform_strut_force->get_friction() * delta);
    printf("Strut friction force is now %d\n", _uniform_strut_force->get_friction());
}

void SoftBunnyThingyDingy::Usage(){
	printf("SPH Controls:\n");
	printf("  c/C: Decrease/Increase coefficient of restitution for box collisions\n");
	printf("  e: Emit? new bunny?\n");
	printf("  g/G: Increase/Decrease gravity strength\n");
	printf("  l: Switch to Leapfrog solver\n");
	printf("  L: Switch to Sixth Order solver\n");
	printf("  r: Reset the simulation\n");
    printf("  s/S increase/decrease strut force\n");
	printf("  u/U: Print this usage information\n");
    printf("  v/V: incrase/decrease strut friction\n");
}

void SoftBunnyThingyDingy::_draw_tris(){
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
	for (size_t i=0; i<_tris_to_draw.size(); i++){
		Color col = _tri_colors[i % _tri_colors.size()];
		glColor3f(col.X(), col.Y(), col.Z());
		const Triangle& tri = _tris_to_draw[i];
		glBegin(GL_TRIANGLES);
			glVertex3d(tri.v0.X(), tri.v0.Y(), tri.v0.Z());
			glVertex3d(tri.v1.X(), tri.v1.Y(), tri.v1.Z());
			glVertex3d(tri.v2.X(), tri.v2.Y(), tri.v2.Z());
		glEnd();
	}
}

void SoftBunnyThingyDingy::_draw_particles(){
	glColor3d(1.0, 0.0, 0.0);
	glPointSize(8.0f);
	glBegin(GL_POINTS);
	for (size_t i=0; i<_dsd->n_particles(); i++){
		Vector pos = _dsd->get_position(i);
		glVertex3f(pos.X(), pos.Y(), pos.Z());
	}
	glEnd();
}
