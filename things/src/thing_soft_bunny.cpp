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
	_create_uniform_soft_body_from_obj("");

    // And now our systems, forces and collision surfaces
	_force_system = std::make_shared<ForceSystem>();
	_solver_system = create_gi_solver_system();
	_box = create_collision_surface();
	_collision_handler = create_collision_handler();
	_collision_handler->register_collision_surface(_box);
	_initialize_box_collision_surface(bounds);
	_gravity_force = std::make_shared<SimpleGravityForce>(Vector(0.0, -9.81, 0.0));
	_uniform_strut_force = std::make_shared<UniformStrutForce>(20.0, 0.3);
    _force_system->add_forces(_gravity_force, _uniform_strut_force);
	
    
	// And now that the init is basically done. Let's build the solvers
    _box->set_restitution( 0.99 );
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
	_draw_box();
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
			break;
		}
		case 'S':{
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
			break;
		}
		case 'V': {
			break;
		}
		default:
			break;
	}
}

void SoftBunnyThingyDingy::_emit_particles(const size_t n){
    _create_uniform_soft_body_from_obj("");
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
	_box->set_restitution( _box->get_restitution() + delta );
	printf("New coefficient of restitution is %f\n", _box->get_restitution());
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


void SoftBunnyThingyDingy::_create_uniform_soft_body_from_obj(const std::string& file_name){
    printf("Hard coding file for now \n");
    std::filesystem::path current_dir = std::filesystem::path(__FILE__).parent_path();
    std::filesystem::path obj_file_path = current_dir / "../../models/bunny_superlo_scaled.obj";

    // we are just going to read it every time for now
    ObjReader<Vector> r(obj_file_path);
    printf("Reading obj from file %s\n", obj_file_path.string().c_str());
    auto verts = r.get_verts();
    auto faces = r.get_faces();
    const size_t n_starting_particles = _dsd->n_particles();
    for (size_t i = 0; i < r.get_verts().size(); i++){
        _dsd->add();
        _dsd->set_position(i + n_starting_particles, verts[i]);
        _dsd->set_velocity(i + n_starting_particles, ParticleEmitter::generate_random_bounded_vector(0.5, 10.0));
    }
    // And now the connections:
    _dsd->connect_all_particles_in_range(n_starting_particles, _dsd->n_particles());
}

void SoftBunnyThingyDingy::Usage(){
	printf("SPH Controls:\n");
	printf("  c/C: Decrease/Increase coefficient of restitution for box collisions\n");
	printf("  e: Emit? new bunny?\n");
	printf("  g/G: Increase/Decrease gravity strength\n");
	printf("  l: Switch to Leapfrog solver\n");
	printf("  L: Switch to Sixth Order solver\n");
	printf("  r: Reset the simulation\n");
	printf("  u/U: Print this usage information\n");
}


void SoftBunnyThingyDingy::_initialize_box_collision_surface(const AABB& bounds){
	// Let's start by just defining the 8 points of the box
	Vector bll = Vector(bounds.lower_left().X(), bounds.lower_left().Y(), bounds.lower_left().Z());
	Vector blf = Vector(bounds.lower_left().X(), bounds.lower_left().Y(),  bounds.upper_right().Z());
	Vector brl = Vector(bounds.upper_right().X(), bounds.lower_left().Y(), bounds.lower_left().Z());
	Vector brf = Vector(bounds.upper_right().X(), bounds.lower_left().Y(),  bounds.upper_right().Z());
	Vector tll = Vector(bounds.lower_left().X(),  bounds.upper_right().Y(), bounds.lower_left().Z());
	Vector tlf = Vector(bounds.lower_left().X(),  bounds.upper_right().Y(),  bounds.upper_right().Z());
	Vector trl = Vector(bounds.upper_right().X(),  bounds.upper_right().Y(), bounds.lower_left().Z());
	Vector trf = Vector(bounds.upper_right().X(),  bounds.upper_right().Y(),  bounds.upper_right().Z());

	// Now let's create the 12 triangles that make up the box
	// All the normals will point into the box
	Triangle b1 = Triangle(bll, brf, brl);
	Triangle b2 = Triangle(bll, blf, brf);
	_box->add_collision_object(std::make_shared<CollisionTriangle>(b1));
	_box->add_collision_object(std::make_shared<CollisionTriangle>(b2));
	Triangle t1 = Triangle(tll, trl, trf);
	Triangle t2 = Triangle(tll, trf, tlf);
	_box->add_collision_object(std::make_shared<CollisionTriangle>(t1));
	_box->add_collision_object(std::make_shared<CollisionTriangle>(t2));
	Triangle l1 = Triangle(bll, tlf, blf);
	Triangle l2 = Triangle(bll, tll, tlf);
	_box->add_collision_object(std::make_shared<CollisionTriangle>(l1));
	_box->add_collision_object(std::make_shared<CollisionTriangle>(l2));
	Triangle r1 = Triangle(brl, brf, trf);
	Triangle r2 = Triangle(brl, trf, trl);
	_box->add_collision_object(std::make_shared<CollisionTriangle>(r1));
	_box->add_collision_object(std::make_shared<CollisionTriangle>(r2));
	Triangle back1 = Triangle(bll, brl, trl);
	Triangle back2 = Triangle(bll, trl, tll);
	_box->add_collision_object(std::make_shared<CollisionTriangle>(back1));
	_box->add_collision_object(std::make_shared<CollisionTriangle>(back2));
	Triangle front1 = Triangle(blf, trf, brf);
	Triangle front2 = Triangle(blf, tlf, trf);
	_box->add_collision_object(std::make_shared<CollisionTriangle>(front1));
	_box->add_collision_object(std::make_shared<CollisionTriangle>(front2));
	
	_tris_to_draw.push_back(b1);
	_tris_to_draw.push_back(b2);
	_tris_to_draw.push_back(t1);
	_tris_to_draw.push_back(t2);
	_tris_to_draw.push_back(l1);
	_tris_to_draw.push_back(l2);
	_tris_to_draw.push_back(r1);
	_tris_to_draw.push_back(r2);
	_tris_to_draw.push_back(back1);
	_tris_to_draw.push_back(back2);
	_tris_to_draw.push_back(front1);
	_tris_to_draw.push_back(front2);
}

void SoftBunnyThingyDingy::_draw_box(){
	// Draw all of the triangles except the last two in the box collision surface
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
