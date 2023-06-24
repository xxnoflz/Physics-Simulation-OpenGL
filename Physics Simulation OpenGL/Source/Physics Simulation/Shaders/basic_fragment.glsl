#version 330 core
out vec4 PixelColor;

in vec2 TexCoord;

uniform sampler2D objectTexture;

void main(){
	PixelColor = texture(objectTexture, TexCoord);
}
