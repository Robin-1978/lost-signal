#version 330

in vec2 fragTexCoord;
in vec4 fragColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;
uniform float threshold;

out vec4 finalColor;

void main() {
 vec3 source = texture(texture0, fragTexCoord).rgb;
 float brightness = max(source.r, max(source.g, source.b));
 float contribution = smoothstep(threshold * 0.72, threshold, brightness);
 finalColor = vec4(source * contribution, 1.0) * colDiffuse * fragColor;
}
