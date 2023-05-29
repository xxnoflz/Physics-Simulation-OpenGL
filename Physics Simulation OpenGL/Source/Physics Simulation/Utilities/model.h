#pragma once
#include <glad/glad.h>

#include <glm/glm.hpp>
#include <glm/gtx/hash.hpp>

#include <vector>
#include <unordered_set>
#include <cstdint>
#include <string>
#include <string_view>

#include "../Render/renderer.h"

namespace Utilities {

	class Model {
	public:
		struct Face {
			std::vector<glm::vec3> vertices;
			glm::vec3 normal;
		};

		Model(const std::vector<glm::vec4>& vertices = {}, const std::vector<uint32_t>& indices = {}, const std::vector<glm::vec3>& normals = {}, 
			const std::vector<uint32_t>& normalIndices = {}, std::string_view shader_name = "");

		void Draw(Render::Renderer* render);
		std::string_view GetShader() const;

		const std::vector<glm::vec4>& GetVertices() const;
		const std::vector<glm::vec3>& GetNormals() const;
		const std::vector<Face>& GetFaces() const;
	private:
		GLuint m_VAO;
		GLuint m_VBO;
		GLuint m_EBO;

		std::string m_shader_name;

		std::vector<glm::vec4> m_vertices;
		std::vector<uint32_t> m_indices;
		std::vector<glm::vec3> m_normals;
		std::vector<uint32_t> m_normalIndices;
		std::vector<Utilities::Model::Face> m_faces;

		void Init();
		void UpdateFaces();
	};

}