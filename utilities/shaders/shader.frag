#version 150 core
in vec4 vertInColor, vertInPosition, vertInNormal;
out vec4 outColor;

const vec3 lightPos = vec3(0.0, 0.0, -100.0);
const vec3 diffuseColor = vec3(1.0, 1.0, 1.0);
const vec3 specColor = vec3(1.0, 1.0, 1.0);
const vec3 ambient = vec3(0.5, 0.5, 0.5);

void main() {
	
	vec3 normVec = vec3(vertInNormal.x, vertInNormal.y, vertInNormal.z);
	vec3 vertPos = vec3(vertInPosition) / vertInPosition.w;
	vec3 lightDir = normalize(lightPos - vertPos);
	vec3 reflectDir = reflect(-lightDir, normVec);
	vec3 viewDir = normalize(-vertPos);

	float lambertian = max(dot(lightDir,normVec), 0.0);
	float specular = 0.0;

	if(lambertian > 0.0) {
		float specAngle = max(dot(reflectDir, viewDir), 0.0);
		specular = pow(specAngle, 4.0);
	}

	vec3 colorScale = lambertian*diffuseColor + specular*specColor + ambient;
	outColor = vec4(vertInColor.x * colorScale.x, vertInColor.y * colorScale.y, vertInColor.z * colorScale.z, 1.0);
}