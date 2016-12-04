#version 420

uniform mat4 mvpMat;
uniform float pointSize;
in vec3 position;

void main() {
	gl_Position = mvpMat*vec4(position,1);
	gl_PointSize = pointSize;
}