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
		struct Edge {
			glm::vec3 first_point;
			glm::vec3 second_point;

			bool operator==(const Edge& other) const {
				return ((first_point == other.first_point
					&& second_point == other.second_point)) ||
					((first_point == other.second_point
						&& second_point == other.first_point));
			}
		};
		struct Face {
			std::vector<glm::vec3> vertices;
			glm::vec3 normal;
			std::vector<Edge> edges;
		};

		Model(const std::vector<glm::vec4>& render_vertices = { }, const std::vector<glm::vec2>& render_texture_uv = { }, const std::vector<uint32_t>& render_indices = { },
			const std::vector<glm::vec4>& vertices = { }, const std::vector<uint32_t>& verticesIndices = { },
			const std::vector<glm::vec3>& normals = { }, const std::vector<uint32_t>& normalIndices = { },
			std::string_view shader_name = "");

		void Draw(Render::Renderer* render);
		std::string_view GetShader() const;

		const std::vector<glm::vec4>& GetVertices() const;
		const std::vector<glm::vec3>& GetNormals() const;
		const std::vector<Face>& GetFaces() const;
	private:
		GLuint m_VAO;
		GLuint m_VBO;
		GLuint m_VBO_texture;
		GLuint m_EBO;

		std::string m_shader_name;

		std::vector<glm::vec4> m_render_vertices;
		std::vector<glm::vec2> m_render_texture_uv;
		std::vector<uint32_t> m_render_indices;

		std::vector<glm::vec4> m_collision_vertices;
		std::vector<uint32_t> m_collision_verticesIndices;
		std::vector<glm::vec3> m_collision_normals;
		std::vector<uint32_t> m_collision_normalIndices;
		std::vector<Utilities::Model::Face> m_faces;

		void Init();
		void InitializeFaces();
	};

}