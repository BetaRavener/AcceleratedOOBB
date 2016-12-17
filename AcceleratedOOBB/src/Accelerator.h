#pragma once
#ifndef ACCELEREATOR_H
#define ACCELEREATOR_H
#include <vector>
#include <glm/detail/type_vec3.hpp>
#include <memory>

#include "OOBB.h"

namespace cl
{
	class Buffer;
	class Context;
	class Device;
	class CommandQueue;
}


class Accelerator
{
public:
	Accelerator();

	cl::Buffer Accelerator::computeCovarianceMatrix(int workGroupSize, cl::Buffer &dataBuffer, int inputSize) const;
	std::pair<glm::vec3, glm::vec3> computeMinMax(cl::Buffer &points, cl::Buffer &eigens, int inputSize, int workGroupSize) const;
	std::pair<cl::Buffer, std::vector<float>> computeEigenVector(cl::Buffer &covarianceMatrix) const;
	OOBB mainRun(std::vector<glm::vec3> &input, int workGroupSize);
	std::pair<cl::Buffer, glm::vec3> computeMean(std::vector<glm::vec3> &input, int workGroupSize) const;
	void Accelerator::centerPoints(cl::Buffer & points, int workGroupSize, int inputSize, glm::vec3 & centroid) const;

private:
	int threadCount;
	std::shared_ptr<cl::Device> device;
	std::shared_ptr<cl::Context> context;
	std::shared_ptr<cl::CommandQueue> queue;
};

#endif