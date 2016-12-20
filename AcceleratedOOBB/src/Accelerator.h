#pragma once
#ifndef ACCELEREATOR_H
#define ACCELEREATOR_H
#include <vector>
#include <glm/detail/type_vec3.hpp>
#include <memory>

#include "OOBB.h"
#include "glmext.h"

namespace cl
{
	class Buffer;
	class Context;
	class Device;
	class CommandQueue;
	class Kernel;
}

class ProgramCL;


class Accelerator
{
public:
	Accelerator();
	~Accelerator();

	cl::Buffer Accelerator::computeCovarianceMatrix(int workGroupSize, cl::Buffer &dataBuffer, int inputSize) const;
	cl::Buffer computeMinMax(cl::Buffer &points, cl::Buffer &eigens, int inputSize, int workGroupSize) const;
	cl::Buffer computeEigenVector(cl::Buffer &covarianceMatrix) const;
	OOBB mainRun(std::vector<glm::vec3> &input, int workGroupSize);
	std::pair<cl::Buffer, glm::vec3> computeMean(std::vector<glm::vec3> &input, int workGroupSize) const;
	void Accelerator::centerPoints(cl::Buffer & points, int workGroupSize, int inputSize, glm::vec3 & centroid) const;

	std::vector<std::vector<int>> sidepodals(std::vector<glm::vec3ext> normals, std::vector<std::pair<int, int>> edges, int workGroupSize) const;

private:
	int threadCount;
	std::shared_ptr<cl::Device> device;
	std::shared_ptr<cl::Context> context;
	std::shared_ptr<cl::CommandQueue> queue;
	std::shared_ptr<ProgramCL> program;
	std::shared_ptr<cl::Kernel> sumKernel, centerKernel, covKernel;
	std::shared_ptr<cl::Kernel> covReductionKernel, eigenKernel;
	std::shared_ptr<cl::Kernel> projKernel, projReductionKernel;
	std::shared_ptr<cl::Kernel> sidepodalKernel;
};

#endif