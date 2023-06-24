#pragma once
#include <glad/glad.h>
#include <cstdint>

namespace Utilities {

	class Texture {
	public:
		Texture(unsigned char* data = nullptr, int width = 0, int height = 0, GLint channels = 0);

		void Use() const;
	private:
		GLuint m_ID;
		uint32_t m_width;
		uint32_t m_height;
		GLint m_channels;

		void Init(unsigned char* data);
	};

};