#include "shader.h"

Utilities::Shader::Shader(std::string_view vertexCode, std::string_view fragmentCode) :
	m_shader_id(), m_uniformCache()
{
	CreateShader(vertexCode, fragmentCode);
}

void Utilities::Shader::CreateShader(std::string_view vertexCode, std::string_view fragmentCode) {
	const char* vertexCodeStr{ vertexCode.data() };
	GLuint vertexShader{ glCreateShader(GL_VERTEX_SHADER) };
	glShaderSource(vertexShader, 1, &vertexCodeStr, nullptr);

	const char* fragmentCodeStr{ fragmentCode.data() };
	GLuint fragmentShader{ glCreateShader(GL_FRAGMENT_SHADER) };
	glShaderSource(fragmentShader, 1, &fragmentCodeStr, nullptr);

	m_shader_id = glCreateProgram();
	glAttachShader(m_shader_id, vertexShader);
	glAttachShader(m_shader_id, fragmentShader);
	glLinkProgram(m_shader_id);

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
}

void Utilities::Shader::Use() const {
	glUseProgram(m_shader_id);
}

GLuint Utilities::Shader::GetID() const {
	return m_shader_id;
}

GLint Utilities::Shader::GetUniformLocation(std::string_view name) {
	auto elementIterator{ m_uniformCache.find(name) };
	if (elementIterator != m_uniformCache.end())
		return elementIterator->second;

	GLint location{ glGetUniformLocation(m_shader_id, name.data()) };
	m_uniformCache[name] = location;
	return location;
}

void Utilities::Shader::SetVec3(std::string_view name, const glm::vec3& value) {
	glUniform3fv(GetUniformLocation(name), 1, &value[0]);
}
void Utilities::Shader::SetMat4(std::string_view name, const glm::mat4& value) {
	glUniformMatrix4fv(GetUniformLocation(name), 1, GL_FALSE, &value[0][0]);
}
