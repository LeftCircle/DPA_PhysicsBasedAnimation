#ifndef _OBJ_READER_H
#define _OBJ_READER_H


// include std::ifstream
#include <fstream>
#include <string>
#include <vector>
#include <ranges>

class ObjReader{
public:
	template <typename Vec3>
	static std::vector<Vec3> get_verts(const std::string& filename){
		std::ifstream in(filename);
		
		std::vector<std::string> lines;
		std::string line;
		while (std::getline(in, line)){
			lines.push_back(line);
		}

		auto filter_func = [](const std::string& l) -> bool {
			return l.substr(0, 2) == "v ";
		};
		auto transform_func = [](const std::string& l) -> Vec3 {
			std::istringstream ss(l.substr(2));
			double x, y, z;
			ss >> x >> y >> z;
			return Vec3(x, y, z);
		};

		auto result = lines
			| std::views::filter(filter_func)
			| std::views::transform(transform_func);
		
		return std::vector<Vec3>(result.begin(), result.end());
	} 
};


#endif