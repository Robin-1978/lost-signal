#version 330

in vec3 fragPosition;
in vec2 fragTexCoord;
in vec4 fragColor;
in vec3 fragNormal;

uniform sampler2D texture0;
uniform vec4 colDiffuse;
uniform vec4 colSpecular;
uniform vec3 viewPos;
uniform vec3 lightDirection;
uniform vec3 lightColor;
uniform vec3 ambient;
uniform vec3 pointLight0Position;
uniform vec3 pointLight0Color;
uniform vec3 pointLight1Position;
uniform vec3 pointLight1Color;
uniform vec3 fogColor;
uniform float fogStart;
uniform float fogEnd;
uniform mat4 lightVP;
uniform sampler2D shadowMap;
uniform int shadowMapResolution;
uniform float shadowStrength;
// A non-zero value selects world-space tiled sampling for SIM floor
// materials.  Regular GLB assets continue to use their authored UVs.
uniform vec2 floorTextureTiling;

out vec4 finalColor;

void main() {
 vec3 normal = normalize(fragNormal);
 if (!gl_FrontFacing) normal = -normal;
 vec3 toLight = normalize(-lightDirection);
 float diffuse = max(dot(normal, toLight), 0.0);
 float halfLambert = diffuse * 0.82 + 0.18;
 vec3 viewDirection = normalize(viewPos - fragPosition);
 vec3 halfDirection = normalize(toLight + viewDirection);
 float metallic = clamp(colSpecular.r, 0.0, 1.0);
 float roughness = clamp(colSpecular.g, 0.04, 1.0);
 float emissive = clamp(colSpecular.b, 0.0, 1.0);
 float shininess = mix(96.0, 7.0, roughness);
 float specular = pow(max(dot(normal, halfDirection), 0.0), shininess)
 * mix(0.30, 0.035, roughness);
 vec3 point0Vector = pointLight0Position - fragPosition;
 vec3 point1Vector = pointLight1Position - fragPosition;
 float point0Distance = length(point0Vector);
 float point1Distance = length(point1Vector);
 float point0Attenuation = pow(clamp(1.0 - point0Distance / 4.8, 0.0, 1.0), 2.0);
 float point1Attenuation = pow(clamp(1.0 - point1Distance / 5.8, 0.0, 1.0), 2.0);
 float point0Diffuse = 0.25 + 0.75 * max(dot(normal, normalize(point0Vector)), 0.0);
 float point1Diffuse = 0.25 + 0.75 * max(dot(normal, normalize(point1Vector)), 0.0);
 vec3 localLight = pointLight0Color * point0Attenuation * point0Diffuse
 + pointLight1Color * point1Attenuation * point1Diffuse;
 vec2 sampleTexCoord = fragTexCoord;
 if (floorTextureTiling.x > 0.0 && floorTextureTiling.y > 0.0) {
  sampleTexCoord = fract(fragPosition.xz * floorTextureTiling);
 }
 vec4 texel = texture(texture0, sampleTexCoord) * colDiffuse * fragColor;
 vec4 fragPosLightSpace = lightVP * vec4(fragPosition, 1.0);
 vec3 shadowCoords = fragPosLightSpace.xyz / max(fragPosLightSpace.w, 0.0001);
 shadowCoords = shadowCoords * 0.5 + 0.5;
 float shadow = 0.0;
 if (shadowCoords.x >= 0.0 && shadowCoords.x <= 1.0
 && shadowCoords.y >= 0.0 && shadowCoords.y <= 1.0
 && shadowCoords.z >= 0.0 && shadowCoords.z <= 1.0) {
 float bias = max(0.0011 * (1.0 - diffuse), 0.00018);
 vec2 texelSize = vec2(1.0 / float(max(shadowMapResolution, 1)));
 for (int x = -1; x <= 1; ++x) {
 for (int y = -1; y <= 1; ++y) {
 float sampledDepth = texture(shadowMap,
 shadowCoords.xy + vec2(float(x), float(y)) * texelSize).r;
 shadow += shadowCoords.z - bias > sampledDepth ? 1.0 : 0.0;
 }
 }
 shadow /= 9.0;
 }
 float directVisibility = 1.0 - shadow * shadowStrength;
 vec3 f0 = mix(vec3(0.04), texel.rgb, metallic);
 vec3 diffuseBase = texel.rgb * (1.0 - metallic * 0.62);
 vec3 lit = diffuseBase * (ambient + lightColor * halfLambert * directVisibility + localLight)
 + f0 * specular * (lightColor * directVisibility + localLight)
 + texel.rgb * emissive * 1.65;
 float distanceToCamera = length(viewPos - fragPosition);
 float fog = smoothstep(fogStart, fogEnd, distanceToCamera) * 0.72;
 finalColor = vec4(mix(lit, fogColor, fog), texel.a);
}
