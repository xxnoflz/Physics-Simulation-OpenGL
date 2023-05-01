#pragma once
#include <fstream>
#include <sstream>
#include <string_view>

#include <map>
#include <vector>

#include "../Model Parser/ModelParser.h"

#include "shader.h"
#include "model.h"

namespace Utilities {

	class ResourceManager {
	public:
		static void LoadShader(std::string_view vertexPath, std::string_view fragmentPath, std::string_view shader_name);
		static Shader& GetShader(std::string_view shader_name);
		static void UseShader(std::string_view shader_name);
		static Shader& GetCurrentShader();
		static std::string_view GetCurrentShaderName();

		static void LoadModel(std::string_view modelPath, std::string_view model_name, std::string_view shader_name);
		static Model& GetModel(std::string_view model_name);
	private:
		static std::map<std::string_view, Shader> m_Shaders;
		static std::map<std::string_view, Model> m_Models;

		static std::string m_currentShader;
	};

}