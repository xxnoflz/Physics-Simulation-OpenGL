#include "texture.h"

Utilities::Texture::Texture(unsigned char* data, int width, int height, GLint channels) :
	m_ID(), m_width(width), m_height(height), m_channels(channels)
{
	Init(data);
}

void Utilities::Texture::Use() const {
	glBindTexture(GL_TEXTURE_2D, m_ID);
}

void Utilities::Texture::Init(unsigned char* data) {
	if(!data)
		return;

	glGenTextures(1, &m_ID);
	glBindTexture(GL_TEXTURE_2D, m_ID);

	glTexImage2D(GL_TEXTURE_2D, 0, m_channels, m_width, m_height, 0, m_channels, GL_UNSIGNED_BYTE, data);
	glGenerateMipmap(GL_TEXTURE_2D);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glBindTexture(GL_TEXTURE_2D, 0);
}