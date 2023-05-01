#pragma once
#include <glad/glad.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <string_view>

#include "../Utilities/shader.h"

namespace Render {

	class Renderer {
	public:
		enum Offsets {
			Projection = 0,
			View = sizeof(glm::mat4)
		};

		Renderer();

		void CreateBuffer();
		void BindShader(Utilities::Shader& shader, std::string_view name);

		void UpdateBuffer(const glm::mat4& matrix, GLintptr offset);

		void Draw(GLuint VAO, uint32_t vertices_count);
		void DrawIndices(GLuint VAO, uint32_t vertices_count);
	private:
		GLuint m_UBO_Matrices;
	};

}