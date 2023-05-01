#pragma once
#include <glad/glad.h>

#include <glm/glm.hpp>

#include <string_view>
#include <unordered_map>

namespace Utilities {

	class Shader {
	public:
		Shader(std::string_view vertexCode = "", std::string_view fragmentCode = "");

		GLuint GetID() const;
		void Use() const;
		void SetVec3(std::string_view name, const glm::vec3& value);
		void SetMat4(std::string_view name, const glm::mat4& value);
	private:
		GLuint m_shader_id;
		std::unordered_map<std::string_view, GLint> m_uniformCache;

		void CreateShader(std::string_view vertexCode, std::string_view fragmentCode);
		GLint GetUniformLocation(std::string_view name);
	};

}