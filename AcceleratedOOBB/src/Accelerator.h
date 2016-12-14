#pragma once
#ifndef ACCELEREATOR_H
#define ACCELEREATOR_H
#include <vector>
#include <glm/detail/type_vec3.hpp>
#include "OOBB.h"

namespace cl
{
	class Buffer;
	class Context;
	class Device;
}

class Accelerator
{
public:
	void run();
	std::pair<cl::Buffer, cl::Buffer> computeCovarianceMatrix(std::vector<glm::vec3> &input, int workGroupSize, cl::Device &device, cl::Context &context);
	std::pair<glm::vec3, glm::vec3> computeMinMax(cl::Buffer &points, cl::Buffer &eigens, int inputSize, int workGroupSize, cl::Device &device, cl::Context &context);
	cl::Buffer computeEigenVector(cl::Buffer &covarianceMatrix, cl::Device &device, cl::Context &context);
	OOBB mainRun(std::vector<glm::vec3> &input, int workGroupSize);

};

#endif