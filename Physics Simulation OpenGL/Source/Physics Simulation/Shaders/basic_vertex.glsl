#version 330 core
layout(location = 0) in vec4 aVertexPosition;

layout(std140) uniform Matrices {
	uniform mat4 projection;
	uniform mat4 view;
};

uniform mat4 model;

void main() {
	gl_Position = projection * view * model * aVertexPosition;
}