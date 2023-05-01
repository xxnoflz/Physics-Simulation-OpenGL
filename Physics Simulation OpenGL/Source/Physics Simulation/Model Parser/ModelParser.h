#pragma once
#include <fstream>
#include <sstream>
#include <string>

#include <vector>

#include <glm/glm.hpp>

class ModelParser {
public:
	struct SingleIndex {
		uint32_t vertex;
		uint32_t texture;
		uint32_t normal;
	};

	struct Indices {
		SingleIndex first;
		SingleIndex second;
		SingleIndex third;
	};
	struct ParsedModel {
		std::vector<glm::vec4> p_vertices;
		std::vector<glm::vec3> p_textureCoords;
		std::vector<glm::vec3> p_normals;
		std::vector<Indices> p_indices;
	};

	static ParsedModel LoadOBJ(const std::string filePath);
};