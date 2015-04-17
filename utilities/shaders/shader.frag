#version 150 core
in vec4 fragInColor, fragInPosition, fragInNormal, fragInLightPos;
out vec4 outColor;

const vec3 diffuseColor = vec3(1.0, 1.0, 1.0);
const vec3 specColor = vec3(1.0, 1.0, 1.0);
const vec3 ambient = vec3(0.5, 0.5, 0.5);

void main() {
	
	vec3 lightPos = vec3(fragInLightPos.x, fragInLightPos.y, fragInLightPos.z);

	vec3 normVec = vec3(fragInNormal.x, fragInNormal.y, fragInNormal.z);
	vec3 vertPos = vec3(fragInPosition) / fragInPosition.w;
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
	outColor = vec4(fragInColor.x * colorScale.x, fragInColor.y * colorScale.y, fragInColor.z * colorScale.z, 1.0);
}