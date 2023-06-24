#include "resource_manager.h"

std::map<std::string_view, Utilities::Shader> Utilities::ResourceManager::m_Shaders;
std::map<std::string_view, Utilities::Model> Utilities::ResourceManager::m_Models;
std::map<std::string_view, Utilities::Texture> Utilities::ResourceManager::m_Textures;
std::string Utilities::ResourceManager::m_currentShader;
std::string Utilities::ResourceManager::m_currentTexture;

void Utilities::ResourceManager::LoadShader(std::string_view vertexPath, std::string_view fragmentPath, std::string_view shader_name) {
	std::ifstream vertexFile{ vertexPath.data() };
	std::stringstream vertexStream{};
	vertexStream << vertexFile.rdbuf();
	std::string vertexCode{ vertexStream.str() };
	vertexFile.close();

	std::ifstream fragmentFile{ fragmentPath.data() };
	std::stringstream fragmentStream{};
	fragmentStream << fragmentFile.rdbuf();
	std::string fragmentCode{ fragmentStream.str() };
	fragmentFile.close();

	m_Shaders[shader_name] = { vertexCode, fragmentCode };
}

Utilities::Shader& Utilities::ResourceManager::GetShader(std::string_view shader_name) {
	return m_Shaders[shader_name];
}

void Utilities::ResourceManager::UseShader(std::string_view shader_name) {
	if(m_currentShader == shader_name)
		return;
	m_Shaders[shader_name].Use();
	m_currentShader = shader_name;
}

Utilities::Shader& Utilities::ResourceManager::GetCurrentShader() {
	return m_Shaders[m_currentShader];
}

std::string_view Utilities::ResourceManager::GetCurrentShaderName() {
	return m_currentShader;
}

void Utilities::ResourceManager::LoadTexture(std::string_view texture_path, std::string_view texture_name) {
	int width;
	int height;
	int channels;
	unsigned char* data{ stbi_load(texture_path.data(), &width, &height, &channels, 0) };
	GLint gl_channels{ };

	switch(channels) {
		case 1:
			gl_channels = GL_RED;
			break;
		case 2:
			gl_channels = GL_RG;
			break;
		case 3:
			gl_channels = GL_RGB;
			break;
		case 4:
			gl_channels = GL_RGBA;
			break;
	}; //Maybe change to map

	m_Textures[texture_name] = { data, width, height, gl_channels };
}

void Utilities::ResourceManager::UseTexture(std::string_view texture_name) {
	if(m_currentTexture == texture_name)
		return;
	m_Textures[texture_name].Use();
	m_currentTexture = texture_name;
}

void Utilities::ResourceManager::LoadModel(std::string_view modelRenderPath, std::string_view modelCollisionPath, std::string_view model_name, std::string_view shader_name) {
	ModelParser::ParsedModel modelRender{ ModelParser::LoadOBJ(modelRenderPath.data()) };
	std::vector<glm::vec2> render_texture_uv{};
	std::vector<uint32_t> renderIndices{};
	for(const auto& [first, second, third] : modelRender.p_indices) {
		renderIndices.push_back(first.vertex - 1);
		renderIndices.push_back(second.vertex - 1);
		renderIndices.push_back(third.vertex - 1);
	}
	for(const auto& uv : modelRender.p_textureCoords)
		render_texture_uv.push_back(uv);

	ModelParser::ParsedModel modelCollision{ ModelParser::LoadOBJ(modelCollisionPath.data()) };
	std::vector<uint32_t> collisionIndices{};
	std::vector<uint32_t> collision_normal_indices{};
	for(const auto& [first, second, third] : modelCollision.p_indices) {
		collisionIndices.push_back(first.vertex - 1);
		collisionIndices.push_back(second.vertex - 1);
		collisionIndices.push_back(third.vertex - 1);

		collision_normal_indices.push_back(first.normal - 1);
		collision_normal_indices.push_back(second.normal - 1);
		collision_normal_indices.push_back(third.normal - 1);
	}

	m_Models[model_name] = { modelRender.p_vertices, render_texture_uv, renderIndices,
							modelCollision.p_vertices, collisionIndices,
							modelCollision.p_normals, collision_normal_indices,
							shader_name };
}

Utilities::Model& Utilities::ResourceManager::GetModel(std::string_view model_name) {
	return m_Models[model_name];
}
