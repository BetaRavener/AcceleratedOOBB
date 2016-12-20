#include "Camera.h"
#include <cmath>
#include "glm/geometric.hpp"

static const float M_PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;
static const float M_PI_2 = M_PI / 2;
static const float _2_M_PI = 2 * M_PI;
static const float _3_M_PI_2 = 3 * M_PI / 2;

glm::vec3 RotateX(glm::vec3 vec, float radians)
{
	auto tempY = vec.y;
	vec.y = (tempY * cos(radians) + vec.z * sin(radians));
	vec.z = (tempY * -sin(radians) + vec.z * cos(radians));
	return vec;
}

glm::vec3  RotateY(glm::vec3 vec, float radians)
{
	auto tempX = vec.x;
	vec.x = (tempX * cos(radians) + vec.z * -sin(radians));
	vec.z = (tempX * sin(radians) + vec.z * cos(radians));
	return vec;
}

Camera::Camera() {
	Reset();
	RotationSensitivity = 0.01;
	ZoomSensitivity = 1.3;
	ZoomInLimit = 1;
	ZoomOutLimit = 50;
}

void Camera::Reset() {
	eye = glm::vec3(0, 0, 10);
	rotations = glm::vec3(0, 0, 0);
	lookAt = glm::vec3(0, 0, 0);
	up = glm::vec3(0, 1, 0);
	invertedY = true;
	changed = true;
}

void Camera::SetLooakAt(glm::vec3& pos)
{
	auto moveVec = pos - lookAt;
	lookAt += moveVec;
	eye += moveVec;
	changed = true;
}

void Camera::RotateA(const glm::vec3 &vec) {
	rotations = glm::vec3(0, 0, 0);
	Rotate(vec.x, vec.y);
	changed = true;
}

void Camera::Rotate(float x, float y) {
	// Convert movement vector into axis rotations.
	auto rotX = y * RotationSensitivity * (invertedY ? -1 : 1);
	auto rotY = -x * RotationSensitivity;

	rotations += glm::vec3(rotX, rotY, 0);

	//check overruns
	while (rotations.y > _2_M_PI)
		rotations.y -= _2_M_PI;
	while (rotations.y < 0.0f)
		rotations.y += _2_M_PI;
	if (rotations.x > M_PI_2 - gimLockBuffer)
		rotations.x = (M_PI_2 - gimLockBuffer);
	else if (rotations.x < -M_PI_2 + gimLockBuffer)
		rotations.x = -(M_PI_2 + gimLockBuffer);

	auto reversedLookAtVec = glm::vec3(0, 0, 1);
	reversedLookAtVec = RotateX(reversedLookAtVec, rotations.x);
	reversedLookAtVec = RotateY(reversedLookAtVec, rotations.y);

	up = glm::vec3(0, 1, 0);
	up = RotateX(up, rotations.x);
	up = RotateY(up, rotations.y);

	auto cameraDistance = glm::distance(lookAt, eye);
	eye = lookAt + reversedLookAtVec * cameraDistance;

	changed = true;
}

void Camera::ZoomBy(float zoomFactor) {
	zoomFactor = zoomFactor*ZoomSensitivity;
	if (zoomFactor < 0)
		zoomFactor = 1 / abs(zoomFactor);

	auto reversedLookAtVec = eye - lookAt;
	auto zoom = glm::length(reversedLookAtVec);
	reversedLookAtVec = glm::normalize(reversedLookAtVec);
	zoom *= zoomFactor;
	if (!isnan(ZoomOutLimit))
		zoom = glm::min(zoom, static_cast<float>(ZoomOutLimit));
	if (!isnan(ZoomInLimit))
		zoom = glm::max(zoom, static_cast<float>(ZoomInLimit));

	eye = lookAt + reversedLookAtVec * zoom;
}

