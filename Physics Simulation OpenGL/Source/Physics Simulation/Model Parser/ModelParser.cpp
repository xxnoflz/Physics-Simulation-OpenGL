#include "ModelParser.h"

ModelParser::ParsedModel ModelParser::LoadOBJ(const std::string filePath) {
	ParsedModel parsedModel{};

	std::ifstream objFile{ filePath, std::ifstream::in };
	if (objFile.fail())
		return ModelParser::ParsedModel();

	std::stringstream objStream{};
	objStream << objFile.rdbuf();
	objFile.close();
	
	std::string currentLine{};
	while (std::getline(objStream, currentLine)) {
		std::istringstream lineIterator{ currentLine };

		std::string elementCharacter{};
		lineIterator >> elementCharacter;

		glm::vec4 coordinate_parse{ 0.0f, 0.0f, 0.0f, 1.0f };
		Indices indice_parse{};

		if (elementCharacter == "v") { //Geometric vertices
			lineIterator >> coordinate_parse.x >> coordinate_parse.y >> coordinate_parse.z >> coordinate_parse.w;
			parsedModel.p_vertices.push_back(coordinate_parse);
		}
		else if (elementCharacter == "vt") { //Texture coordinates
			lineIterator >> coordinate_parse.x >> coordinate_parse.y >> coordinate_parse.z;
			parsedModel.p_textureCoords.push_back(coordinate_parse);
		}
		else if (elementCharacter == "vn") { //Vertex normals
			lineIterator >> coordinate_parse.x >> coordinate_parse.y >> coordinate_parse.z;
			parsedModel.p_normals.push_back(glm::normalize(glm::vec3(coordinate_parse)));
		}
		else if (elementCharacter == "f") {
			if (sscanf_s(lineIterator.str().c_str(), "f %d %d %d", 
				&indice_parse.first.vertex, &indice_parse.second.vertex, &indice_parse.third.vertex) == 3 || //Vertex Indices
			sscanf_s(lineIterator.str().c_str(), "f %d/%d %d/%d %d/%d", 
				&indice_parse.first.vertex, &indice_parse.first.texture, 
				&indice_parse.second.vertex, &indice_parse.second.texture,
				&indice_parse.third.vertex, &indice_parse.third.texture) == 6 ||							//Vertex texture coordinate indices
			sscanf_s(lineIterator.str().c_str(), "f %d/%d/%d %d/%d/%d %d/%d/%d", 
				&indice_parse.first.vertex, &indice_parse.first.texture, &indice_parse.first.normal,
				&indice_parse.second.vertex, &indice_parse.second.texture, &indice_parse.second.normal,
				&indice_parse.third.vertex, &indice_parse.third.texture, &indice_parse.third.normal) == 9 ||//Vertex normal indices
			sscanf_s(lineIterator.str().c_str(), "f %d//%d %d//%d %d//%d", 
				&indice_parse.first.vertex, &indice_parse.first.normal,
				&indice_parse.second.vertex, &indice_parse.second.normal,
				&indice_parse.third.vertex, &indice_parse.third.normal) == 6)								//Vertex normal indices without texture coordinate indices
			{						
				parsedModel.p_indices.push_back(indice_parse);
			}
		}
	}
	return parsedModel;
}