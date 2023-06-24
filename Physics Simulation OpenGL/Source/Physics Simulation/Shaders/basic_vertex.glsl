#version 330 core
layout(location = 0) in vec4 aVertexPosition;
layout(location = 1) in vec2 aTexturePosition;

layout(std140) uniform Matrices {
	uniform mat4 projection;
	uniform mat4 view;
};

uniform mat4 model;

out vec2 TexCoord;

void main() {
	gl_Position = projection * view * model * aVertexPosition;
	TexCoord = aTexturePosition;
}