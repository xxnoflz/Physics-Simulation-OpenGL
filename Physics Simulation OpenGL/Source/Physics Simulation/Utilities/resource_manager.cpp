#include "resource_manager.h"

std::map<std::string_view, Utilities::Shader> Utilities::ResourceManager::m_Shaders;
std::map<std::string_view, Utilities::Model> Utilities::ResourceManager::m_Models;
std::string Utilities::ResourceManager::m_currentShader;

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
	if (m_currentShader == shader_name)
		return;
	m_Shaders[shader_name].Use();
	m_currentShader = shader_name;
}

Utilities::Shader& Utilities::ResourceManager::GetCurrentShader() {
	return m_Shaders[m_currentShader];
}

std::string_view Utilities::ResourceManager::GetCurrentShaderName(){
	return m_currentShader;
}

void Utilities::ResourceManager::LoadModel(std::string_view modelPath, std::string_view model_name, std::string_view shader_name) {
	ModelParser::ParsedModel model{ ModelParser::LoadOBJ(modelPath.data()) };
	std::vector<uint32_t> indices{};
	std::vector<uint32_t> normal_indices{};
	for (auto [first, second, third] : model.p_indices) {
		indices.push_back(first.vertex - 1);
		indices.push_back(second.vertex - 1);
		indices.push_back(third.vertex - 1);

		normal_indices.push_back(first.normal - 1);
		normal_indices.push_back(second.normal - 1);
		normal_indices.push_back(third.normal - 1);
	}
	m_Models[model_name] = { model.p_vertices, indices, model.p_normals, normal_indices, shader_name };
}

Utilities::Model& Utilities::ResourceManager::GetModel(std::string_view model_name) {
	return m_Models[model_name];
}
