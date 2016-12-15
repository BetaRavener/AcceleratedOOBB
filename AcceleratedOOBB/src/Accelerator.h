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
	cl::Buffer Accelerator::computeCovarianceMatrix(int workGroupSize, cl::Device &device, cl::Context &context, cl::Buffer &dataBuffer, int inputSize);
	std::pair<glm::vec3, glm::vec3> computeMinMax(cl::Buffer &points, cl::Buffer &eigens, int inputSize, int workGroupSize, cl::Device &device, cl::Context &context);
	std::pair<cl::Buffer, std::vector<float>> computeEigenVector(cl::Buffer &covarianceMatrix, cl::Device &device, cl::Context &context);
	OOBB mainRun(std::vector<glm::vec3> &input, int workGroupSize);
	std::pair<cl::Buffer, glm::vec3> computeMean(std::vector<glm::vec3> &input, int workGroupSize, cl::Device &device, cl::Context &context);
	void Accelerator::centerPoints(cl::Buffer & points, int workGroupSize, cl::Device &device, cl::Context &context, int inputSize, glm::vec3 & centroid);

private:
	int threadCount;
};

#endif