#ifndef _OBJ_READER_H
#define _OBJ_READER_H


// include std::ifstream
#include <fstream>
#include <string>
#include <vector>
#include <ranges>

#include "vector.h"

template <typename Vec3>
class ObjReader{
public:
	ObjReader(const std::string& filename) {
		std::ifstream in(filename);

		std::vector<std::string> lines;
		std::string line;
		while (std::getline(in, line)){
			lines.push_back(line);
		}
	
		auto verts = lines 
			| std::views::filter([](const std::string& l) { return l.substr(0, 2) == "v "})
			| std::views::transform(line_to_vec);
		
		auto faces = lines 
			| std::views::filter([](const std::string& l) { return l.substr(0, 2) == "f "})
			| std::views::transform([](const std::string& l){
				std::istringstream ss(l.substr(2));
				int x y z;
				ss >> x >> y >> z;
				return cato::Vec3i(x, y, z);
			})
	}


	static std::vector<Vec3> get_verts(const std::string& filename){
		std::ifstream in(filename);
		
		std::vector<std::string> lines;
		std::string line;
		while (std::getline(in, line)){
			lines.push_back(line);
		}

		auto result = lines
			| std::views::filter([](const std::string& l) -> bool { return l.substr(0, 2) == "v "; })
			| std::views::transform(line_to_vec);
		
		return std::vector<Vec3>(result.begin(), result.end());
	}
private:
	ObjReader() = delete;

	static Vec3 line_to_vec(const std::string& l) {
		std::istringstream ss(l.substr(2));
		double x, y, z;
		ss >> x >> y >> z;
		return Vec3(x, y, z);
	}

	std::vector<Vec3> verts;
	std::vector<cato::Vec3i> faces; 
};


#endif