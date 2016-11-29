#version 420

uniform mat4 mvpMat;
in vec3 position;

void main() {
	gl_Position = mvpMat*vec4(position,1);
	gl_PointSize = 5.0;
}