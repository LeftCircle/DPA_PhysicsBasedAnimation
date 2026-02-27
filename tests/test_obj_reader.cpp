#include "catch_helpers.h"
#include "obj_reader.h"

#include <Vector.h>
#include <vector>

using namespace pba;

TEST_CASE("test get verts from obj file"){
    const std::string obj_file_path = "../models/bunny_superlo_scaled.obj";

    std::vector<Vector> verts = ObjReader::get_verts<Vector>(obj_file_path);

    REQUIRE(verts.size() == 1920);
}