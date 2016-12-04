#include "Scene.h"
#include "glm/vec2.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include <vector>
#include "Generator.h"

using namespace std;

Scene::Scene() {
	_pointCloudSize = 1000;
}

vector<glm::vec3> Scene::boxToVertices(glm::vec3 min, glm::vec3 max)
{
	auto backBotLeft = glm::vec3(min);
	auto backBotRight = glm::vec3(max.x, min.y, min.z);
	auto backTopLeft = glm::vec3(min.x, max.y, min.z);
	auto backTopRight = glm::vec3(max.x, max.y, min.z);
	auto frontBotLeft = glm::vec3(min.x, min.y, max.z);
	auto frontBotRight = glm::vec3(max.x, min.y, max.z);
	auto frontTopLeft = glm::vec3(min.x, max.y, max.z);
	auto frontTopRight = glm::vec3(max);

	vector<glm::vec3> vertices = {
		backBotLeft, frontBotLeft, backBotRight, frontBotRight, // Bottom side
		backTopRight, frontTopRight, // Right side
		backTopLeft, frontTopLeft, // Top side
		frontTopLeft, frontBotRight, // Reset
		frontBotRight, frontTopRight, frontBotLeft, frontTopLeft, // Front side
		backBotLeft, backTopLeft, // Left side
		backBotRight, backTopRight // Back side		
	};

	return vertices;
}

void Scene::init() {
	_camera.Reset();

	updateProjectionMatrix();
	updateViewMatrix();

	// Create shader program
	string prefix = "../shaders/";
	auto bgVertShader = compileShader(GL_VERTEX_SHADER, loadFile(prefix + "vert.shader"));
	auto bgFragShader = compileShader(GL_FRAGMENT_SHADER, loadFile(prefix + "frag.shader"));
	_program = linkShader(2, bgVertShader, bgFragShader);

	// Use created shader program
	glUseProgram(_program);

	// Enable depth test and shader-defined point size
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_PROGRAM_POINT_SIZE);

	// Get the location of the attributes and uniforms
	auto positionAttrib = glGetAttribLocation(_program, "position");
	_mvpMatIdx = glGetUniformLocation(_program, "mvpMat");
	_colorIdx = glGetUniformLocation(_program, "color");

	// Create Vertex Array Objects that save configuration of bindings and pointers
	glGenVertexArrays(1, &_pointsVao);
	glGenVertexArrays(1, &_boxVao);

	// Create Vector Buffer Objects that will store the vertices in GPU
	GLuint pointVBO, boxVBO;
	glGenBuffers(1, &pointVBO);
	glGenBuffers(1, &boxVBO);

	// Generate point cloud
	auto generator = Generator(glm::vec3(-1, -1, -1), glm::vec3(1, 1, 1), glm::vec3(1, 1, 1));
	auto pointCloudVertices = generator.CreatePointCloud(_pointCloudSize);

	auto cpu = Cpu();
	cpu.CreateOOBB(pointCloudVertices);

	glBindVertexArray(_pointsVao);
	glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*pointCloudVertices.size(), &pointCloudVertices[0], GL_STATIC_DRAW);
	glVertexAttribPointer(positionAttrib, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
	glEnableVertexAttribArray(positionAttrib);

	// Assemble bounding box
	auto boxVertices = boxToVertices(glm::vec3(-1, -1, -1), glm::vec3(1, 1, 1));

	glBindVertexArray(_boxVao);
	glBindBuffer(GL_ARRAY_BUFFER, boxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * boxVertices.size(), &boxVertices[0], GL_STATIC_DRAW);
	glVertexAttribPointer(positionAttrib, 3, GL_FLOAT, GL_FALSE, 0, nullptr);	
	glEnableVertexAttribArray(positionAttrib);

	// Unbind shader program
	glUseProgram(0);
}

void Scene::draw(){
	// Clear color and depth buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Bind rendering shader program
	glUseProgram(_program);

	// Set ModelViewProjection matrix
	auto mvpMat = _projMat * _viewMat * _modelMat;
	glUniformMatrix4fv(_mvpMatIdx, 1, GL_FALSE, glm::value_ptr(mvpMat));

	// Set color for points
	auto color = glm::vec4(1, 0, 0, 1);
	glUniform4fv(_colorIdx, 1, glm::value_ptr(color));

	// Use VAO for point cloud to render it
	glBindVertexArray(_pointsVao);
	glPolygonMode(GL_FRONT, GL_FILL);
	glPolygonMode(GL_BACK, GL_FILL);
	glDrawArrays(GL_POINTS, 0, _pointCloudSize);

	// Set color for bounding box
	color = glm::vec4(0, 1, 1, 1);
	glUniform4fv(_colorIdx, 1, glm::value_ptr(color));

	// Use VAO for box to render it
	glBindVertexArray(_boxVao);
	if (_wireframeBox)
	{
		// Draw as wireframe
		glPolygonMode(GL_FRONT, GL_LINE);
		glPolygonMode(GL_BACK, GL_LINE);
	}
	else
	{
		glPolygonMode(GL_FRONT, GL_FILL);
		glPolygonMode(GL_BACK, GL_FILL);
	}
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 18);

	// Unbind program
	glUseProgram(0);
}

void Scene::onResize(int w, int h)
{
	updateProjectionMatrix();
}

void Scene::updateProjectionMatrix() {
	glViewport(0, 0, width, height);
	_projMat = glm::perspective(45.0f, static_cast<float>(width) / static_cast<float>(height), 0.1f, 100.0f);
}

void Scene::updateViewMatrix() {
	_viewMat = glm::lookAt(_camera.GetEye(), _camera.GetLookAt(), _camera.GetUp());
}

void Scene::onMouseMove(int dx, int dy, int x, int y)
{
	if (_rightButtonDown)
	{
		_camera.Rotate(dx, dy);
		updateViewMatrix();
	}
}

void Scene::onKeyPress(SDL_Keycode key, Uint16 mod){
	switch (key) {
		case SDLK_ESCAPE:
			quit();
		case SDLK_w:
			_wireframeBox = !_wireframeBox;
		default: break;
	}
}

void Scene::onMouseWheel(int delta)
{
	_camera.ZoomBy(delta);
	updateViewMatrix();
}

void Scene::onMousePress(Uint8 button, int x, int y){
	if (button == 3)
	{
		_rightButtonDown = true;
	}
}

void Scene::onMouseRelease(Uint8 button, int x, int y) {
	if (button == 3)
	{
		_rightButtonDown = false;
	}
}

int main(int /*argc*/, char ** /*argv*/)
{
	Scene app;
	return app.run();
}