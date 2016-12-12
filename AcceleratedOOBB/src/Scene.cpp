#include "Scene.h"
#include "glm/vec2.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include <vector>
#include "Generator.h"
#include "Accelerator.h"
#include <thread>
#include "Model.h"

using namespace std;

Scene::Scene() {
	_pointCloudSize = 10000;
	_wireframeBox = false;
	_pointSize = 5;
}

vector<glm::vec3> Scene::buildBoundingBox(glm::vec3 center, glm::vec3 axes[], float minimums[], float maximums[])
{
	auto min = glm::vec3(center);
	auto max = glm::vec3(center);
	float ranges[3];

	for (auto i = 0; i < 3; i++)
	{
		min += axes[i] * minimums[i];
		max += axes[i] * maximums[i];
		ranges[i] = maximums[i] - minimums[i];
	}

	auto backBotLeft = glm::vec3(min);
	auto backBotRight = glm::vec3(min + axes[0] * ranges[0]);
	auto backTopLeft = glm::vec3(min + axes[1] * ranges[1]);
	auto backTopRight = glm::vec3(max - axes[2] * ranges[2]);
	auto frontBotLeft = glm::vec3(min + axes[2] * ranges[2]);
	auto frontBotRight = glm::vec3(max - axes[1] * ranges[1]);
	auto frontTopLeft = glm::vec3(max - axes[0] * ranges[0]);
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

void runCL(std::vector<glm::vec3> points)
{
	auto acc = Accelerator();
	acc.run2(points, 256);
}

void runCL2(std::vector<glm::vec3> points, std::vector<glm::vec3> eigens)
{
	auto acc = Accelerator();
	acc.run3(points, eigens, 256);
}

void Scene::prepareScene(std::vector<glm::vec3>& pointCloudVertices)
{
	auto cpu = Cpu();
	auto oobb = cpu.CreateOOBB(pointCloudVertices);
	auto eigens = cpu.CreateEigens(pointCloudVertices);

	/*std::thread second(runCL2, pointCloudVertices, eigens);
	second.detach();*/

	glUseProgram(_program);

	glBindVertexArray(_pointsVao);
	glBindBuffer(GL_ARRAY_BUFFER, _pointsVbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*pointCloudVertices.size(), &pointCloudVertices[0], GL_STATIC_DRAW);

	// Assemble bounding box
	auto boxVertices = buildBoundingBox(oobb.center, oobb.axes, oobb.minimums, oobb.maximums);

	glBindVertexArray(_boxVao);
	glBindBuffer(GL_ARRAY_BUFFER, _boxVbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * boxVertices.size(), &boxVertices[0], GL_STATIC_DRAW);

	glUseProgram(0);

	_pointCloudSize = pointCloudVertices.size();
}

std::vector<glm::vec3> Scene::loadModel(std::string fileName, float scale)
{
	auto pointCloudVertices = Model::load(fileName, scale);
	prepareScene(pointCloudVertices);
	return pointCloudVertices;
}

void Scene::init() {
	_camera.Reset();

	updateProjectionMatrix();
	updateViewMatrix();

	// Create shader program
	string prefix = "../Shaders/";
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
	_pointSizeIdx = glGetUniformLocation(_program, "pointSize");

	// Create Vertex Array Objects that save configuration of bindings and pointers
	glGenVertexArrays(1, &_pointsVao);
	glGenVertexArrays(1, &_boxVao);

	// Create Vector Buffer Objects that will store the vertices in GPU
	glGenBuffers(1, &_pointsVbo);
	glGenBuffers(1, &_boxVbo);

	glBindVertexArray(_pointsVao);
	glBindBuffer(GL_ARRAY_BUFFER, _pointsVbo);
	glVertexAttribPointer(positionAttrib, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
	glEnableVertexAttribArray(positionAttrib);

	glBindVertexArray(_boxVao);
	glBindBuffer(GL_ARRAY_BUFFER, _boxVbo);
	glVertexAttribPointer(positionAttrib, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
	glEnableVertexAttribArray(positionAttrib);

	// Unbind shader program
	glUseProgram(0);

	auto pointCloudVertices = loadModel("bunny.data", 10);

	std::thread first(runCL, pointCloudVertices);
	first.detach();
}

glm::vec3 Scene::colorFromRgb(uint8_t r, uint8_t g, uint8_t b) const
{
	return glm::normalize(glm::vec3(r / 255.0f, g / 255.0f, b / 255.0f));
}

void Scene::draw(){
	// Clear color and depth buffers
	glClearColor(0.1, 0.1, 0.1, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Bind rendering shader program
	glUseProgram(_program);

	// Set ModelViewProjection matrix
	auto mvpMat = _projMat * _viewMat * _modelMat;
	glUniformMatrix4fv(_mvpMatIdx, 1, GL_FALSE, glm::value_ptr(mvpMat));

	// Set color for points
	auto color = glm::vec4(colorFromRgb(39, 183, 0), 1);
	glUniform4fv(_colorIdx, 1, glm::value_ptr(color));

	// Set point size
	glUniform1f(_pointSizeIdx, _pointSize);

	// Use VAO for point cloud to render it
	glBindVertexArray(_pointsVao);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDrawArrays(GL_POINTS, 0, _pointCloudSize);

	// Set color for bounding box
	color = glm::vec4(colorFromRgb(107, 0, 86), 1);
	glUniform4fv(_colorIdx, 1, glm::value_ptr(color));

	// Use VAO for box to render it
	glBindVertexArray(_boxVao);
	if (_wireframeBox)
	{
		// Draw as wireframe
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	}
	else
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
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
	auto generatorAxis = glm::normalize(glm::vec3(1, 1, 1));
	auto mins = glm::vec3(-1, -0.8, -0.6);
	auto maxs = glm::vec3(1, 0.8, 0.6);
	auto generator = Generator(mins, maxs, generatorAxis);

	switch (key) {
		case SDLK_ESCAPE:
			quit();
			break;
		case SDLK_w:
			_wireframeBox = !_wireframeBox;
			break;
		case SDLK_q:
			if (_pointSize > 1.0f)
				_pointSize -= 1.0f;
			break;
		case SDLK_e:
			if (_pointSize < 10.0f)
				_pointSize += 1.0f;
			break;
		case SDLK_1:
			loadModel("dragon_hi.data", 10);
			break;
		case SDLK_2:
			loadModel("bunny.data", 10);
			break;
		case SDLK_3:
			loadModel("budha.data", 10);
			break;
		case SDLK_4: {
			auto pointCloudVertices = generator.CreatePointCloud(50000);
			prepareScene(pointCloudVertices);
			break;
		}
		case SDLK_5: {
			auto pointCloudVertices = generator.CreatePointCloud(500000);
			prepareScene(pointCloudVertices);
			break;
		}
		case SDLK_6: {
			auto pointCloudVertices = generator.CreatePointCloud(5000000);
			prepareScene(pointCloudVertices);
			break;
		}
		case SDLK_7:
			loadModel("teapot.data", 2);
			break;
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
