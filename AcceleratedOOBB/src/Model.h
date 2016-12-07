#pragma once
#ifndef MODEL_H
#define MODEL_H
#include <vector>
#include <glm/detail/type_vec3.hpp>

class Model
{
public:
	static std::vector<glm::vec3> load(std::string fileName, float scale = 1);
};

#endif