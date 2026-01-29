#ifndef _RENDER_SERVER_H
#define _RENDER_SERVER_H

#include <vector>

#include "shapes.h"
#include "dynamical_state_data.h"
#include "Color.h"

namespace pba{

// a singleton render server class. 
class RenderServer {
public:
	static RenderServer& get_instance() {
		static RenderServer instance;
		return instance;
	}

	void add_triangle(const Triangle& tri, const Color& color) {
		_triangles.push_back(tri);
		_triangle_colors.push_back(color);
	}

	void register_dynamical_state_data(DynamicalStateData_sp dsd) {
		_dsd_list.push_back(dsd);
	}

	void render();

private:
	RenderServer() = default;
	~RenderServer() = default;

	// Delete copy constructor and assignment operator to enforce singleton property
	RenderServer(const RenderServer&) = delete;
	RenderServer& operator=(const RenderServer&) = delete;

	std::vector<DynamicalStateData_sp> _dsd_list;
	std::vector<Triangle> _triangles;
	std::vector<Color> _triangle_colors;
};

RenderServer& get_render_server_instance() {
	return RenderServer::get_instance();
}

} // end namespace pba

#endif