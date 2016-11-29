#pragma once
#ifndef GENERATOR_H
#define GENERATOR_H
#include <glm/detail/type_vec3.hpp>
#include <vector>

class Generator
{
public:
	Generator(glm::vec3 min, glm::vec3 max, glm::vec3 axis);
	std::vector<glm::vec3> CreatePointCloud(int count) const;
private:
	glm::vec3 _min;
	glm::vec3 _max;
	glm::vec3 _axis;
	glm::vec3 _up;
	glm::vec3 _right;
};

#endif