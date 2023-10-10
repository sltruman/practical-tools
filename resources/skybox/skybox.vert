#version 330 core

in vec4 p3d_Vertex;

out vec3 fTexCoord;

uniform mat4 p3d_ViewMatrix;
uniform mat4 p3d_ProjectionMatrix;

void main()
{
    fTexCoord = vec3(p3d_Vertex);

    mat4 View = mat4(mat3(p3d_ViewMatrix));
    vec4 pos = p3d_ProjectionMatrix * View * p3d_Vertex;

    gl_Position = pos.xyww;
}
