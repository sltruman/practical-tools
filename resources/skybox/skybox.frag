#version 330 core

in vec3 fTexCoord;

out vec4 fColor;

uniform samplerCube TexSkybox;

void main()
{
    fColor = texture(TexSkybox, fTexCoord);
}
