#version 150
in vec4 position;
uniform mat4 transformMatrix;
void main() 
	gl_Position = transformMatrix * position;
}