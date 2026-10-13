#version 330

in vec2 fragTexCoord;
in vec4 fragColor;

uniform sampler2D texture0;
uniform sampler2D bloomTexture;
uniform vec4 colDiffuse;
uniform float exposure;
uniform float bloomStrength;
uniform float vignetteStrength;

out vec4 finalColor;

vec3 aces_film(vec3 value) {
 return clamp((value * (2.51 * value + 0.03))
 / (value * (2.43 * value + 0.59) + 0.14), 0.0, 1.0);
}

void main() {
 vec3 hdr = texture(texture0, fragTexCoord).rgb;
 vec3 bloom = texture(bloomTexture, fragTexCoord).rgb;
 vec3 mapped = aces_film((hdr + bloom * bloomStrength) * exposure);
 float distanceFromCenter = length(fragTexCoord - vec2(0.5));
 float vignette = 1.0 - smoothstep(0.30, 0.76, distanceFromCenter) * vignetteStrength;
 mapped *= vignette;
 finalColor = vec4(mapped, 1.0) * colDiffuse * fragColor;
}
