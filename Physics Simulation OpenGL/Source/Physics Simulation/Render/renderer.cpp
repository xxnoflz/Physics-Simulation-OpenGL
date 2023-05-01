#include "renderer.h"

Render::Renderer::Renderer() : 
	m_UBO_Matrices() 
{

}

void Render::Renderer::CreateBuffer() {
	if (m_UBO_Matrices)
		return;
	glGenBuffers(1, &m_UBO_Matrices);
	glBindBuffer(GL_UNIFORM_BUFFER, m_UBO_Matrices);
	glBufferData(GL_UNIFORM_BUFFER, 2 * sizeof(glm::mat4), nullptr, GL_STATIC_DRAW);
	glBindBufferRange(GL_UNIFORM_BUFFER, 0, m_UBO_Matrices, 0, 2 * sizeof(glm::mat4));
	glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void Render::Renderer::BindShader(Utilities::Shader& shader, std::string_view name) {
	glUniformBlockBinding(shader.GetID(), glGetUniformBlockIndex(shader.GetID(), name.data()), 0);
}

void Render::Renderer::UpdateBuffer(const glm::mat4& matrix, GLintptr offset) {
	glBindBuffer(GL_UNIFORM_BUFFER, m_UBO_Matrices);
	glBufferSubData(GL_UNIFORM_BUFFER, offset, sizeof(matrix), &matrix[0][0]);
	glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void Render::Renderer::Draw(GLuint VAO, uint32_t vertices_count) {
	glBindVertexArray(VAO);
	glDrawArrays(GL_TRIANGLES, 0, vertices_count);
	glBindVertexArray(0);
}

void Render::Renderer::DrawIndices(GLuint VAO, uint32_t vertices_count) {
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, vertices_count, GL_UNSIGNED_INT, nullptr);
	glBindVertexArray(0);
}