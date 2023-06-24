#include "model.h"

Utilities::Model::Model(const std::vector<glm::vec4>& render_vertices, const std::vector<glm::vec2>& render_texture_uv, const std::vector<uint32_t>& render_indices,
	const std::vector<glm::vec4>& vertices, const std::vector<uint32_t>& verticesIndices,
	const std::vector<glm::vec3>& normals, const std::vector<uint32_t>& normalIndices,
	std::string_view shader_name) :
	m_render_vertices(render_vertices), m_render_texture_uv(render_texture_uv), m_render_indices(render_indices),
	m_collision_vertices(vertices), m_collision_verticesIndices(verticesIndices),
	m_collision_normals(normals), m_collision_normalIndices(normalIndices),
	m_shader_name(shader_name) 
{
	Init();
	InitializeFaces();
}

void Utilities::Model::Init() {
	glGenVertexArrays(1, &m_VAO);
	glGenBuffers(1, &m_VBO);
	glGenBuffers(1, &m_VBO_texture);
	glGenBuffers(1, &m_EBO);
	glBindVertexArray(m_VAO);

	glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
	glBufferData(GL_ARRAY_BUFFER, glm::vec4::length() * sizeof(float) * m_render_vertices.size(), m_render_vertices.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * m_render_indices.size(), m_render_indices.data(), GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, glm::vec4::length(), GL_FLOAT, GL_FALSE, glm::vec4::length() * sizeof(float), (void*)0);

	glBindBuffer(GL_ARRAY_BUFFER, m_VBO_texture);
	glBufferData(GL_ARRAY_BUFFER, glm::vec2::length() * sizeof(float) * m_render_texture_uv.size(), m_render_texture_uv.data(), GL_STATIC_DRAW);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, glm::vec2::length(), GL_FLOAT, GL_FALSE, glm::vec2::length() * sizeof(float), (void*)0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void Utilities::Model::InitializeFaces() {
	if(m_collision_verticesIndices.size() % 3 != 0 && m_collision_normalIndices.size() % 3 != 0 && m_collision_verticesIndices.size() != m_collision_normalIndices.size())
		return;

	for(uint32_t iterator{}; iterator < m_collision_verticesIndices.size(); iterator += 6) {
		Face face{};

		face.normal = m_collision_normals[m_collision_normalIndices[iterator]];

		for(uint32_t vertex_iterator{ iterator }; vertex_iterator < iterator + 6; ++vertex_iterator) {
			glm::vec3 element{ m_collision_vertices[m_collision_verticesIndices[vertex_iterator]] };
			if(std::find(face.vertices.begin(), face.vertices.end(), element) == face.vertices.end())
				face.vertices.push_back(element);
		}

		for(uint32_t point_iterator{}; point_iterator < face.vertices.size(); ++point_iterator) {
			glm::vec3 first_point{ face.vertices[point_iterator] };
			glm::vec3 second_point{ face.vertices[(point_iterator + 1) % face.vertices.size()] };

			Edge new_edge{ first_point, second_point };

			if(std::find(face.edges.begin(), face.edges.end(), new_edge) == face.edges.end())
				face.edges.push_back({ first_point, second_point });
		}

		m_faces.push_back(face);
	}
}

void Utilities::Model::Draw(Render::Renderer* render) {
	render->DrawIndices(m_VAO, m_render_indices.size());
}

std::string_view Utilities::Model::GetShader() const {
	return m_shader_name;
}

const std::vector<glm::vec4>& Utilities::Model::GetVertices() const {
	return m_collision_vertices;
}

const std::vector<glm::vec3>& Utilities::Model::GetNormals() const {
	return m_collision_normals;
}

const std::vector<Utilities::Model::Face>& Utilities::Model::GetFaces() const {
	return m_faces;
}