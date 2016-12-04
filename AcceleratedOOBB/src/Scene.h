#pragma once
#ifndef SCENE_H
#define SCENE_H

#include "BaseApp.h"
#include "Camera.h"
#include <glm/matrix.hpp>
#include <vector>

class Scene: public BaseApp{
public:
	Scene();
protected:
	int _pointCloudSize;
	bool _wireframeBox;
	float _pointSize;

	Camera _camera;
	glm::mat4 _projMat;
	glm::mat4 _viewMat;
	glm::mat4 _modelMat;

	GLuint _pointsVao, _boxVao;
	GLuint _program;
	GLuint _mvpMatIdx, _colorIdx, _pointSizeIdx;
	
	bool _rightButtonDown;

	void init() override;
	void draw() override;
	void onResize(int w, int h) override;
	void onMouseMove(int dx, int dy, int x, int y) override;
	void onKeyPress(SDL_Keycode key, Uint16 mod) override;
	void onMouseWheel(int delta) override;
	void onMousePress(Uint8 button, int x, int y) override;
	void onMouseRelease(Uint8 button, int x, int y) override;
private:
	static std::vector<glm::vec3> buildBoundingBox(glm::vec3 center, glm::vec3 axes[], float minimums[], float maximums[]);
	glm::vec3 colorFromRgb(uint8_t r, uint8_t g, uint8_t b) const;
	void updateProjectionMatrix();
	void updateViewMatrix();
};

#endif