#pragma once
#ifndef ACCELEREATOR_H
#define ACCELEREATOR_H
#include <vector>
#include <glm/detail/type_vec3.hpp>
//#include <CL/cl.hpp>

class Accelerator
{
public:
	void run();
	void run2(std::vector<glm::vec3> &input, int workGroupSize);
	void run3(std::vector<glm::vec3> &input, std::vector<glm::vec3> eigens, int workGroupSize);
};

#endif