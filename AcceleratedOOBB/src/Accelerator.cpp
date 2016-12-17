#include "Accelerator.h"

#include "../Clock.h"
#include "Common.h"
#include "Cpu.h"
#include "Helpers.h"
#include "ProgramCL.h"
#include "Platforms.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <iostream>
#include <glm/detail/type_vec3.hpp>
#include <sstream>

#define SELECTED_DEVICE_TYPE CL_DEVICE_TYPE_CPU

#define TIMING_GPU
#define TIMING_CPU
#ifdef TIMING_GPU
#define PROFILE_FLAG CL_QUEUE_PROFILING_ENABLE
#else
#define PROFILE_FLAG 0
#endif

#define MATRIX_W 1024
#define MATRIX_H 1024

double getEventTime(cl::Event i_event)
{
	return double(i_event.getProfilingInfo<CL_PROFILING_COMMAND_END>() - i_event.getProfilingInfo<CL_PROFILING_COMMAND_START>()) * 1e-9;
}

double getEventVectorTime(std::vector<cl::Event> events)
{
	auto acc = 0.;
	for (auto event : events)
		acc += getEventTime(event);

	return acc;
}

std::string formatEventTime(cl::Event event, bool milliseconda = true)
{
	return Clock::FormatTime(getEventTime(event));
}

int geqPow2(int n)
{
	auto x = 1;
	while (x < n)
		x <<= 1;
	return x;
}

int nextGroupSize(int count)
{
	return glm::max(glm::min(geqPow2(count),256), 32);
}

Accelerator::Accelerator()
{
	Platforms::printAllInfos();

	device = std::make_shared<cl::Device>(Platforms::getCpu());

	// check if device is correct
	if (Platforms::getDeviceType(*device) == CL_DEVICE_TYPE_GPU)
	{
		std::cout << "Selected device type: Correct" << std::endl;
	}
	else
	{
		std::cout << "Selected device type: Incorrect" << std::endl;
	}
	std::cout << "Selected device: " << std::endl;
	Platforms::printDeviceInfo(*device);
	std::cout << std::endl;

	//===========================================================================================
	context = std::make_shared<cl::Context>(cl::Context(*device));
	queue = std::make_shared<cl::CommandQueue>(cl::CommandQueue(*context, *device, PROFILE_FLAG));

	cl_uint computeUnitCount;
	size_t retSize;
	Helpers::checkErorCl(
		clGetDeviceInfo((*device)(), CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(cl_uint), static_cast<void*>(&computeUnitCount), &retSize),
		"Getting GPU INFO");

	threadCount = computeUnitCount * 2048 * 2;
}

OOBB Accelerator::mainRun(std::vector<glm::vec3> &input, int workGroupSize)
{
	auto t0 = Clock::Tick();

	auto inputSize = input.size();

	auto bufferAndSum = computeMean(input, 256);

	auto centroid = bufferAndSum.second / static_cast<float>(inputSize);

	centerPoints(bufferAndSum.first, workGroupSize, inputSize, centroid);

	auto covBuffer = computeCovarianceMatrix(workGroupSize, bufferAndSum.first, inputSize);

	auto bufferAndResult = computeEigenVector(covBuffer);

	auto minMax = computeMinMax(bufferAndSum.first, bufferAndResult.first, inputSize, workGroupSize);

	Helpers::checkErorCl(queue->finish(), "clFinish");
	auto t1 = Clock::Tick();
	std::cout << "TIME: Total time: " << Clock::FormatTime(t1 - t0) << std::endl;

	auto result = OOBB();
	result.center = centroid;

	result.maximums[0] = minMax.second.x;
	result.maximums[1] = minMax.second.y;
	result.maximums[2] = minMax.second.z;

	result.minimums[0] = minMax.first.x;
	result.minimums[1] = minMax.first.y;
	result.minimums[2] = minMax.first.z;

	result.axes[0] = glm::vec3(bufferAndResult.second[0], bufferAndResult.second[1], bufferAndResult.second[2]);
	result.axes[1] = glm::vec3(bufferAndResult.second[3], bufferAndResult.second[4], bufferAndResult.second[5]);
	result.axes[2] = glm::vec3(bufferAndResult.second[6], bufferAndResult.second[7], bufferAndResult.second[8]);

	return result;
}

std::pair<cl::Buffer, glm::vec3> Accelerator::computeMean(std::vector<glm::vec3> &input, int workGroupSize) const
{
	auto t0 = Clock::Tick();

	cl_int code;

	auto inputSize = input.size();

	// For Example - it has to be dividable by 3 and by workGroupSize
	auto alignedSize = Helpers::alignSize(inputSize, workGroupSize) * 3;
	if (alignedSize > threadCount)
	{
		// We have to be sure its dividable by 3
		alignedSize = Helpers::alignSize((threadCount / 3) - workGroupSize, workGroupSize) * 3;
	}

	auto groupsCount = alignedSize / workGroupSize;

	auto finalSum = std::vector<float>(3);

	auto alignedInputSize = Helpers::alignSize(inputSize, workGroupSize);
	auto data = std::vector<float>();

	data.resize(3 * alignedInputSize);
	for (auto i = 0; i < 3; i++)
		for (auto j = 0; j < alignedInputSize; j++)
			data[i*alignedInputSize + j] = j < inputSize ? input[j][i] : 0;


	//===========================================================================================

	auto program = ProgramCL(*device, *context, { "../Kernels/points_sum.cl" });
	auto sumKernel = program.getKernel("points_sum");

	//===========================================================================================
	cl_mem_flags flags = CL_MEM_READ_WRITE;
	auto dataBufferSize = data.size() * sizeof(float);
	auto resultBufferSize = groupsCount * sizeof(float);

	auto dataBuffer = cl::Buffer(*context, flags, dataBufferSize);
	auto resultBuffer = cl::Buffer(*context, flags, resultBufferSize);

	//===========================================================================================
	sumKernel.setArg(0, dataBuffer);
	sumKernel.setArg(1, workGroupSize * sizeof(float), nullptr);
	sumKernel.setArg(2, resultBuffer);
	sumKernel.setArg(3, alignedInputSize);
	sumKernel.setArg(4, alignedSize);

	cl::UserEvent write_event(*context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent write_event");
	cl::UserEvent sum_kernel_event(*context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent sum_kernel_event");
	cl::UserEvent reduce_kernel_event(*context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent reduce_kernel_event");
	cl::UserEvent read_event(*context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent read_event");

	//===========================================================================================
	cl::NDRange local(workGroupSize);
	cl::NDRange global(alignedSize);

	queue->enqueueWriteBuffer(dataBuffer, false, 0, dataBufferSize, &data[0], nullptr, &write_event);
	queue->enqueueNDRangeKernel(sumKernel, 0, global, local, nullptr, &sum_kernel_event);

	// It is possible, that all is reduced, but if not, we continue reducing
	inputSize = groupsCount / 3;
	if (inputSize > 1)
	{
		alignedSize = Helpers::alignSize(inputSize, workGroupSize) * 3;
		//workGroupSize = nextWorkGroupSize;

		local = cl::NDRange(workGroupSize);
		global = cl::NDRange(alignedSize);

		auto finalBufferSize = 3 * sizeof(float);
		auto finalBuffer = cl::Buffer(*context, flags, finalBufferSize);

		sumKernel.setArg(0, resultBuffer);
		sumKernel.setArg(1, workGroupSize * sizeof(float), nullptr);
		sumKernel.setArg(2, finalBuffer);
		sumKernel.setArg(3, inputSize);
		sumKernel.setArg(4, alignedSize);

		queue->enqueueNDRangeKernel(sumKernel, 0, global, local, nullptr, &reduce_kernel_event);

		resultBufferSize = finalBufferSize;
	}

	queue->enqueueReadBuffer(resultBuffer, false, 0, resultBufferSize, &finalSum[0], nullptr, &read_event);

	// synchronize queue
#ifdef TIMING_GPU
	Helpers::checkErorCl(queue->finish(), "clFinish");
	std::cout << "TIME: Vertex data write: " << formatEventTime(write_event) << std::endl;
	std::cout << "TIME: Mean computation time: " << formatEventTime(sum_kernel_event) << std::endl;
	std::cout << "TIME: Reduction time: " << (inputSize > 1 ? formatEventTime(reduce_kernel_event) : "skipped") << std::endl;
	std::cout << "TIME: Vertex data read: " << formatEventTime(read_event) << std::endl;
#endif
#ifdef TIMING_CPU
	auto t1 = Clock::Tick();
	std::cout << "TIME: Compute mean CPU: " << Clock::FormatTime(t1 - t0) << std::endl;
#endif

	return std::make_pair(dataBuffer, glm::vec3(finalSum[0], finalSum[1], finalSum[2]));
}

void Accelerator::centerPoints(cl::Buffer & points, int workGroupSize, int inputSize, glm::vec3 & centroid) const
{
	cl_int code;

	auto t0 = Helpers::getTime();

	// For Example - it has to be dividable by 3
	auto alignedSize = Helpers::alignSize(inputSize, workGroupSize) * 3;
	if (alignedSize > threadCount)
	{
		// We have to be sure its dividable by 3
		alignedSize = Helpers::alignSize((threadCount / 3) - workGroupSize, workGroupSize) * 3;
	}

	auto groupsCount = alignedSize / workGroupSize;

	auto alignedInputSize = Helpers::alignSize(inputSize, workGroupSize);

	//===========================================================================================
	auto program = ProgramCL(*device, *context, { "../Kernels/center_points.cl" });
	auto centerKernel = program.getKernel("center_points");

	//===========================================================================================
	centerKernel.setArg(0, points);
	centerKernel.setArg(1, centroid.x);
	centerKernel.setArg(2, centroid.y);
	centerKernel.setArg(3, centroid.z);

	centerKernel.setArg(4, alignedInputSize);
	centerKernel.setArg(5, alignedSize);
	centerKernel.setArg(6, inputSize);

	cl::UserEvent kernel_event(*context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent kernel_event");

	//===========================================================================================
	cl::NDRange local(workGroupSize);
	cl::NDRange global(alignedSize);

	// Run kernel
	queue->enqueueNDRangeKernel(centerKernel, 0, global, local, nullptr, &kernel_event);

#ifdef TIMING_GPU
	Helpers::checkErorCl(queue->finish(), "clFinish");
	std::cout << "TIME: Vertex centering: " << formatEventTime(kernel_event) << std::endl;
#endif
#ifdef TIMING_CPU
	auto t1 = Clock::Tick();
	std::cout << "TIME: Center points CPU: " << Clock::FormatTime(t1 - t0) << std::endl;
#endif
}

cl::Buffer Accelerator::computeCovarianceMatrix(int workGroupSize, cl::Buffer &dataBuffer, int inputSize) const
{
	cl_int code;

	auto t0 = Helpers::getTime();

	auto const pointsCount = inputSize;
	auto alignedSize = Helpers::alignSize(inputSize, workGroupSize);

	auto resultSize = Helpers::ceilDiv(inputSize, workGroupSize);
	auto nextWorkGroupSize = nextGroupSize(resultSize);
	auto nextAlignedSize = Helpers::alignSize(resultSize, nextWorkGroupSize);

	auto covIntermResult = std::vector<float>();
	covIntermResult.resize(6 * nextWorkGroupSize);
	
	//===========================================================================================
	auto program = ProgramCL(*device, *context, { "../Kernels/covariance_mat.cl", "../Kernels/covariance_reduce.cl" });
	auto covKernel = program.getKernel("covariance_matrix");
	auto reductionKernel = program.getKernel("cov_reduce");

	//===========================================================================================
	cl_mem_flags flags = CL_MEM_READ_WRITE;
	auto covIntermBufferSize = covIntermResult.size() * sizeof(float);
	auto covarianceResultBuffer = cl::Buffer(*context, flags, covIntermBufferSize);
	auto covarianceSecondBuffer = cl::Buffer(*context, flags, covIntermBufferSize);

	//===========================================================================================
	covKernel.setArg(0, dataBuffer);
	covKernel.setArg(1, covarianceResultBuffer);
	covKernel.setArg(2, workGroupSize, nullptr);
	covKernel.setArg(3, inputSize);
	covKernel.setArg(4, alignedSize);
	covKernel.setArg(5, nextAlignedSize);

	cl::UserEvent kernel_event(*context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent kernel_event");
	std::vector<cl::Event> reduction_events;

	//===========================================================================================
	cl::NDRange local(workGroupSize);
	cl::NDRange global(6 * alignedSize);

	// Run kernel
	queue->enqueueNDRangeKernel(covKernel, 0, global, local, nullptr, &kernel_event);

	while (resultSize != 1)
	{
		inputSize = resultSize;
		alignedSize = nextAlignedSize;
		workGroupSize = nextWorkGroupSize;
		auto multiplier = 1.0f;

		resultSize = Helpers::ceilDiv(inputSize, workGroupSize);
		if (resultSize > 1)
		{
			nextWorkGroupSize = nextGroupSize(resultSize);
			nextAlignedSize = Helpers::alignSize(resultSize, nextWorkGroupSize);
		}
		else
		{
			nextWorkGroupSize = 0;
			nextAlignedSize = 1;
			multiplier = 1.0f/(pointsCount-1);
		}

		local = cl::NDRange(workGroupSize);
		global = cl::NDRange(6 * alignedSize);

		reductionKernel.setArg(0, covarianceResultBuffer);
		reductionKernel.setArg(1, covarianceSecondBuffer);
		reductionKernel.setArg(2, workGroupSize, nullptr);
		reductionKernel.setArg(3, inputSize);
		reductionKernel.setArg(4, alignedSize);
		reductionKernel.setArg(5, nextAlignedSize);
		reductionKernel.setArg(6, multiplier);

		cl::UserEvent *reduction_event = nullptr;
#ifdef TIMING_GPU
		reduction_events.push_back(cl::UserEvent(*context, &code));
		Helpers::checkErorCl(code, "clCreateUserEvent kernel_event");
#endif

		queue->enqueueNDRangeKernel(reductionKernel, 0, global, local, nullptr, reduction_event);

		Swap(covarianceSecondBuffer, covarianceResultBuffer);
	}

#ifdef TIMING_GPU
	Helpers::checkErorCl(queue->finish(), "clFinish");
	std::cout << "TIME: First covariance pass: " << formatEventTime(kernel_event) << std::endl;
	std::cout << "TIME: Covariance reductions: " << Clock::FormatTime(getEventVectorTime(reduction_events)) << std::endl;
#endif
#ifdef TIMING_CPU
	auto t1 = Clock::Tick();
	std::cout << "TIME: Covariance CPU: " << Clock::FormatTime(t1 - t0) << std::endl;
#endif

	return covarianceResultBuffer;
}

std::pair<cl::Buffer, std::vector<float>> Accelerator::computeEigenVector(cl::Buffer &covarianceMatrix) const
{
	cl_int code;

	auto t0 = Helpers::getTime();

	auto result = std::vector<float>(9);

	//===========================================================================================
	auto program = ProgramCL(*device, *context, { "../Kernels/eigenvector.cl" });
	auto kernel = program.getKernel("compute_eigens");

	//===========================================================================================
	cl_mem_flags flags = CL_MEM_READ_WRITE;

	auto eigensBufferSize = 9 * sizeof(float);
	auto eigensBuffer = cl::Buffer(*context, flags, eigensBufferSize);

	//===========================================================================================
	kernel.setArg(0, covarianceMatrix);
	kernel.setArg(1, eigensBuffer);

	// compute results on host
	cl::UserEvent kernel_event(*context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent kernel_event");
	cl::UserEvent c_event(*context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent c_event");

	//===========================================================================================
	cl::NDRange local(1);
	cl::NDRange global(1);

	// Run kernel
	queue->enqueueNDRangeKernel(kernel, 0, global, local, nullptr, &kernel_event);

	// Read data from GPU
	//queue.enqueueReadBuffer(eigensBuffer, false, 0, eigensBufferSize, &result[0], nullptr, &c_event);

#ifdef TIMING_GPU
	Helpers::checkErorCl(queue->finish(), "clFinish");
	std::cout << "TIME: Eigen vectors: " << formatEventTime(kernel_event) << std::endl;
#endif
#ifdef TIMING_CPU
	auto t1 = Clock::Tick();
	std::cout << "TIME: Eigens CPU: " << Clock::FormatTime(t1 - t0) << std::endl;
#endif

	return std::make_pair(eigensBuffer, result);
}

std::pair<glm::vec3, glm::vec3> Accelerator::computeMinMax(cl::Buffer &points, cl::Buffer &eigens, int inputSize, int workGroupSize) const
{
	cl_int code;

	auto t0 = Helpers::getTime();

	auto alignedSize = Helpers::alignSize(inputSize, workGroupSize);
	int globalCount = alignedSize;
	int groupsCount = alignedSize / workGroupSize;

	// Just to check intermediate values
	auto minMaxResults = std::vector<float>();
	minMaxResults.resize(6 * groupsCount);


	//===========================================================================================
	auto program = ProgramCL(*device, *context, { "../Kernels/projection_mat.cl" });
	auto kernel = program.getKernel("projection_matrix");

	//auto program2 = ProgramCL(gpu_device, context, { "../Kernels/reduction_minmax.cl" });
	auto reduce_kernel = program.getKernel("reduction_minmax");

	//===========================================================================================
	cl_mem_flags flags = CL_MEM_READ_WRITE;
	auto resultBufferSize = 6 * groupsCount * sizeof(float);
	auto resultBuffer = cl::Buffer(*context, flags, resultBufferSize);
	auto resultBuffer2 = cl::Buffer(*context, flags, resultBufferSize);

	//===========================================================================================

	kernel.setArg(0, points);
	kernel.setArg(1, resultBuffer);
	kernel.setArg(2, eigens);
	kernel.setArg(3, 9, nullptr);
	kernel.setArg(4, 6 * workGroupSize, nullptr);
	kernel.setArg(5, inputSize);
	kernel.setArg(6, alignedSize);

	// compute results on host
	cl::UserEvent a_event(*context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent a_event");
	cl::UserEvent b_event(*context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent b_event");
	cl::UserEvent kernel_event(*context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent kernel_event");
	cl::UserEvent c_event(*context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent c_event");

	//===========================================================================================
	cl::NDRange local(workGroupSize);
	cl::NDRange global(alignedSize);

	// Run kernel
	queue->enqueueNDRangeKernel(kernel, 0, global, local, nullptr, &kernel_event);

	inputSize = groupsCount;
	cl::Buffer *inputBuffer = &resultBuffer;
	cl::Buffer *result = &resultBuffer2;
	while (groupsCount > 1)
	{
		alignedSize = Helpers::alignSize(inputSize, workGroupSize);
		globalCount = alignedSize;
		groupsCount = alignedSize / workGroupSize;

		reduce_kernel.setArg(0, *inputBuffer);
		reduce_kernel.setArg(1, *result);
		reduce_kernel.setArg(2, 6 * workGroupSize, nullptr);
		reduce_kernel.setArg(3, inputSize);
		reduce_kernel.setArg(4, alignedSize);

		queue->enqueueNDRangeKernel(reduce_kernel, 0, cl::NDRange(alignedSize), cl::NDRange(workGroupSize), nullptr, &kernel_event);

		inputSize = groupsCount;
		cl::Buffer *tmp = inputBuffer;
		inputBuffer = result;
		result = tmp;
	}

	// Read data from GPU
	//queue.enqueueReadBuffer(*inputBuffer, false, 0, 6 * sizeof(float), &minMaxResults[0], nullptr, &c_event);

#ifdef TIMING_GPU
	Helpers::checkErorCl(queue->finish(), "clFinish");
	std::cout << "TIME: Min/Max projection: " << formatEventTime(kernel_event) << std::endl;
#endif
#ifdef TIMING_CPU
	auto t1 = Clock::Tick();
	std::cout << "TIME: Projection CPU: " << Clock::FormatTime(t1 - t0) << std::endl;
#endif

	auto min = glm::vec3(minMaxResults[0], minMaxResults[1], minMaxResults[2]);
	auto max = glm::vec3(minMaxResults[3], minMaxResults[4], minMaxResults[5]);

	return std::make_pair(min, max);
}

