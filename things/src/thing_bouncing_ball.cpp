#include "thing_bouncing_ball.h"


using namespace pba;


BouncingBallThing::BouncingBallThing(const std::string& nam)
: PbaThingyDingy(nam) {
    // Create the defaults for the dynamical state and all the solvers!
    _dsd = create_dynamical_state_data();
    _force_system = std::make_shared<ForceSystem>();
    _solver_system = create_gi_solver_system();
    _box = create_collision_surface();
    _collision_handler = create_collision_handler();
    _collision_handler->register_collision_surface(_box);
    _initialize_box_collision_surface();
}

void BouncingBallThing::_initialize_box_collision_surface(){
    // Let's create a box by using a set of triangles for collision detection
    const double half_length = 5.0;
    
    // Let's start by just defining the 8 points of the box
    Vector bll = Vector(-half_length, -half_length, -half_length); // bottom left back
    Vector blf = Vector(-half_length, -half_length,  half_length); // bottom left front
    Vector brl = Vector( half_length, -half_length, -half_length); // bottom right back
    Vector brf = Vector( half_length, -half_length,  half_length); // bottom right front
    Vector tll = Vector(-half_length,  half_length, -half_length); // top left back
    Vector tlf = Vector(-half_length,  half_length,  half_length); // top left front
    Vector trl = Vector( half_length,  half_length, -half_length); // top right back
    Vector trf = Vector( half_length,  half_length,  half_length); // top right front

    // Now let's create the 12 triangles that make up the box
    // The front face will be added last so that it's easier to avoid drawing it
    // bottom face
    _box->add_collision_object(std::make_shared<CollisionTriangle>(bll, brl, brf));
    _box->add_collision_object(std::make_shared<CollisionTriangle>(bll, brf, blf));
    // Top face
    _box->add_collision_object(std::make_shared<CollisionTriangle>(tll, tlf, trf));
    _box->add_collision_object(std::make_shared<CollisionTriangle>(tll, trf, trl));
    // Left face
    _box->add_collision_object(std::make_shared<CollisionTriangle>(bll, blf, tlf));
    _box->add_collision_object(std::make_shared<CollisionTriangle>(bll, tlf, tll));
    // Right face
    _box->add_collision_object(std::make_shared<CollisionTriangle>(brl, trl, trf));
    _box->add_collision_object(std::make_shared<CollisionTriangle>(brl, trf, brf));
    // Back face
    _box->add_collision_object(std::make_shared<CollisionTriangle>(bll, tll, trl));
    _box->add_collision_object(std::make_shared<CollisionTriangle>(bll, trl, brl));
    // Front face
    _box->add_collision_object(std::make_shared<CollisionTriangle>(blf, brf, trf));
    _box->add_collision_object(std::make_shared<CollisionTriangle>(blf, trf, tlf));
}

void BouncingBallThing::_draw_box(){
    // Draw all of the triangles except the last two in the box collision surface
    glColor3f(0.7f, 0.7f, 0.7f);
    glBegin(GL_TRIANGLES);
    const auto& collision_objects = _box->get_collision_objects();
}
