#version 330

in vec2 fragTexCoord;
in vec4 fragColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;
uniform vec2 texelSize;
uniform vec2 direction;

out vec4 finalColor;

void main() {
 vec2 offset = texelSize * direction;
 vec3 color = texture(texture0, fragTexCoord).rgb * 0.227027;
 color += texture(texture0, fragTexCoord + offset * 1.384615).rgb * 0.316216;
 color += texture(texture0, fragTexCoord - offset * 1.384615).rgb * 0.316216;
 color += texture(texture0, fragTexCoord + offset * 3.230769).rgb * 0.070270;
 color += texture(texture0, fragTexCoord - offset * 3.230769).rgb * 0.070270;
 finalColor = vec4(color, 1.0) * colDiffuse * fragColor;
}
