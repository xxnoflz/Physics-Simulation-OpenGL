#version 330 core
out vec4 PixelColor;

uniform vec3 objectColor;

void main(){
	PixelColor = vec4(objectColor, 1.0f);
}
