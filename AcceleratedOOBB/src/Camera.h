#pragma once
#ifndef CAMERA_H
#define CAMERA_H

#include <cfloat>
#include <glm/detail/type_vec3.hpp>

#define gimLockBuffer FLT_EPSILON*1E3

class Camera {
public:
	Camera();
	const glm::vec3 &GetEye() const { return eye; }
	const glm::vec3 &GetLookAt() const { return lookAt; }
	const glm::vec3 &GetUp() const { return up; }
	bool IsChanged() const { return changed; }
	void AnnulChanges() { changed = false; }
	void RotateA(const glm::vec3 &vec);
	void Rotate(float x, float y);
	void InvertY() { invertedY = !invertedY; }
	void ZoomBy(float zoomFactor);
	void Reset();
	void SetLooakAt(glm::vec3 &pos);
protected:
	glm::vec3 eye, lookAt, up;
	bool invertedY, changed;
	glm::vec3 rotations;

	double RotationSensitivity;
	double ZoomSensitivity;
	double ZoomInLimit;
	double ZoomOutLimit;
};

#endif