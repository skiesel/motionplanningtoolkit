#version 150
in vec4 position, normal, color;
in mat4 transform;
out vec4 fragInColor, fragInPosition, fragInNormal, fragInLightPos;
uniform mat4 transformMatrix;

const vec4 lightPos = vec4(0.0, 0.0, -100.0, 1.0);

void main() {
	gl_Position = transformMatrix * transform * position;

	fragInColor = color;
	fragInPosition = gl_Position;
	fragInNormal = transformMatrix * transform * normal;
	fragInLightPos = transformMatrix * transform * lightPos;
}