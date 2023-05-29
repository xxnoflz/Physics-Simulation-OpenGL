#include "model.h"

Utilities::Model::Model(const std::vector<glm::vec4>& vertices, const std::vector<uint32_t>& indices, const std::vector<glm::vec3>& normals, 
	const std::vector<uint32_t>& normalIndices, std::string_view shader_name) :
	m_vertices(vertices), m_indices(indices), m_normals(normals), m_normalIndices(normalIndices), m_shader_name(shader_name)
{
	Init();
	UpdateFaces();
}

void Utilities::Model::Init() {
	glGenVertexArrays(1, &m_VAO);
	glGenBuffers(1, &m_VBO);
	glGenBuffers(1, &m_EBO);
	glBindVertexArray(m_VAO);

	glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
	glBufferData(GL_ARRAY_BUFFER, glm::vec4::length() * sizeof(float) * m_vertices.size(), m_vertices.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * m_indices.size(), m_indices.data(), GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, glm::vec4::length(), GL_FLOAT, GL_FALSE, glm::vec4::length() * sizeof(float), (void*)0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void Utilities::Model::UpdateFaces() {
	if(m_indices.size() % 3 != 0 && m_normalIndices.size() % 3 != 0 && m_indices.size() != m_normalIndices.size())
		return;

	for (uint32_t iterator{}; iterator < m_indices.size(); iterator += 6) {
		Face face{};
		face.normal = m_normals[m_normalIndices[iterator]];
		for (uint32_t vertex_iterator{ iterator }; vertex_iterator < iterator + 6; ++vertex_iterator) {
			glm::vec3 element{ m_vertices[m_indices[vertex_iterator]] };
			if(std::find(face.vertices.begin(), face.vertices.end(), element) == face.vertices.end())
				face.vertices.push_back(element);
		}
		m_faces.push_back(face);
	}
}

void Utilities::Model::Draw(Render::Renderer* render) {
	render->DrawIndices(m_VAO, m_indices.size());
}

std::string_view Utilities::Model::GetShader() const {
	return m_shader_name;
}

const std::vector<glm::vec4>& Utilities::Model::GetVertices() const {
	return m_vertices;
}

const std::vector<glm::vec3>& Utilities::Model::GetNormals() const {
	return m_normals;
}

const std::vector<Utilities::Model::Face>& Utilities::Model::GetFaces() const {
	return m_faces;
}