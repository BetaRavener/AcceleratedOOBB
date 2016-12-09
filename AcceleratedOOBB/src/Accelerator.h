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
private:
	//void reduce(cl::Kernel kernel, cl::Buffer bufferA, cl::Buffer bufferB);
};

#endif