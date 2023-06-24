#pragma once
#include <fstream>
#include <sstream>
#include <string_view>

#include <map>
#include <vector>

#include "../Model Parser/ModelParser.h"
#include "../Vendor/stb_image.h"

#include "shader.h"
#include "model.h"
#include "texture.h"

namespace Utilities {

	class ResourceManager {
	public:
		static void LoadShader(std::string_view vertexPath, std::string_view fragmentPath, std::string_view shader_name);
		static Shader& GetShader(std::string_view shader_name);
		static void UseShader(std::string_view shader_name);
		static Shader& GetCurrentShader();
		static std::string_view GetCurrentShaderName();

		static void LoadTexture(std::string_view texture_path, std::string_view texture_name);
		static void UseTexture(std::string_view texture_name);

		static void LoadModel(std::string_view modelRenderPath, std::string_view modelCollisionPath, std::string_view model_name, std::string_view shader_name);
		static Model& GetModel(std::string_view model_name);
	private:
		static std::map<std::string_view, Shader> m_Shaders;
		static std::map<std::string_view, Model> m_Models;
		static std::map<std::string_view, Texture> m_Textures;

		static std::string m_currentShader;
		static std::string m_currentTexture;
	};

}