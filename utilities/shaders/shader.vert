#version 150
in vec4 position, normal, color;
in mat4 transform;
out vec4 vertInColor, vertInPosition, vertInNormal;
uniform mat4 transformMatrix;

void main() {
	gl_Position = transformMatrix * transform * position;

	vertInColor = color;
	vertInPosition = gl_Position;
	vertInNormal = transformMatrix * transform * normal;
}