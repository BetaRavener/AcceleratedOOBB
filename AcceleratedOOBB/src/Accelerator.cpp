#include "Accelerator.h"

#include "Clock.h"
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

#define SELECTED_DEVICE_TYPE CL_DEVICE_TYPE_GPU

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

std::string getEventStatus(cl::Event i_event)
{
	auto status = i_event.getInfo<CL_EVENT_COMMAND_EXECUTION_STATUS>();
	switch (status)
	{
	case CL_COMPLETE:
		return "COMPLETE";
	case CL_SUBMITTED:
		return "SUBMITTED";
	case CL_RUNNING:
		return "RUNNING";
	case CL_QUEUED:
		return "QUEUED";
	}
	return "UNKNOWN_ERROR";
}

double getEventVectorTime(const std::vector<cl::Event>& events)
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

	// check if device is correct
	if (SELECTED_DEVICE_TYPE == CL_DEVICE_TYPE_GPU)
	{
		device = std::make_shared<cl::Device>(Platforms::findPowerfulGpu());
		std::cout << "Selected device type: GPU" << std::endl;
	}
	else
	{
		device = std::make_shared<cl::Device>(Platforms::getCpu());
		std::cout << "Selected device type: CPU" << std::endl;
	}
	std::cout << "Selected device: " << std::endl;
	Platforms::printDeviceInfo(*device);
	std::cout << std::endl;

	//===========================================================================================
	cl_int code;
	context = std::make_shared<cl::Context>(cl::Context(*device));
//	Helpers::checkErorCl(code, "clContext");
	queue = std::make_shared<cl::CommandQueue>(cl::CommandQueue(*context, *device, PROFILE_FLAG));

	cl_uint computeUnitCount;
	size_t retSize;
	Helpers::checkErorCl(
		clGetDeviceInfo((*device)(), CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(cl_uint), static_cast<void*>(&computeUnitCount), &retSize),
		"Getting GPU INFO");

	threadCount = computeUnitCount * 2048 * 2;

	program = std::make_shared<ProgramCL>(ProgramCL(*device, *context, { "../Kernels/points_sum.cl",
		"../Kernels/center_points.cl",
		"../Kernels/covariance_mat.cl",
		"../Kernels/covariance_reduce.cl",
		"../Kernels/eigenvector.cl",
		"../Kernels/projection_mat.cl",
		"../Kernels/sidepodal.cl"}));

	sumKernel = std::make_shared<cl::Kernel>(program->getKernel("points_sum"));
	centerKernel = std::make_shared<cl::Kernel>(program->getKernel("center_points"));
	covKernel = std::make_shared<cl::Kernel>(program->getKernel("covariance_matrix"));
	covReductionKernel = std::make_shared<cl::Kernel>(program->getKernel("cov_reduce"));
	eigenKernel = std::make_shared<cl::Kernel>(program->getKernel("compute_eigens"));
	projKernel = std::make_shared<cl::Kernel>(program->getKernel("projection_matrix"));
	projReductionKernel = std::make_shared<cl::Kernel>(program->getKernel("reduction_minmax"));
	sidepodalKernel = std::make_shared<cl::Kernel>(program->getKernel("sidepodal"));
}

Accelerator::~Accelerator()
{
	sumKernel.reset();
	centerKernel.reset();
	covKernel.reset();
	covReductionKernel.reset();
	eigenKernel.reset();
	projKernel.reset();
	projReductionKernel.reset();
	sidepodalKernel.reset();

	program.reset();

	queue.reset();

	context.reset();

	device.reset();
}

OOBB Accelerator::mainRun(std::vector<glm::vec3> &input, int workGroupSize)
{
	auto t0 = Clock::Tick();

	auto inputSize = input.size();

	auto bufferAndSum = computeMean(input, workGroupSize);

	auto centroid = bufferAndSum.second / static_cast<float>(inputSize);

	centerPoints(bufferAndSum.first, workGroupSize, inputSize, centroid);

	auto covBuffer = computeCovarianceMatrix(workGroupSize, bufferAndSum.first, inputSize);

	std::vector<float> check; check.resize(6);
	queue->enqueueReadBuffer(covBuffer, true, 0, 6 * sizeof(float), &check[0], nullptr, nullptr);

	auto eigensBuffer = computeEigenVector(covBuffer);

	auto minMaxBuffer = computeMinMax(bufferAndSum.first, eigensBuffer, inputSize, workGroupSize);

	cl::Event eigenReadEvent;
	cl::Event minMaxReadEvent;

	// Read eigenvectors from GPU
	std::vector<float> eigenVectors; eigenVectors.resize(9);
	queue->enqueueReadBuffer(eigensBuffer, false, 0, eigenVectors.size() * sizeof(float), &eigenVectors[0], nullptr, &eigenReadEvent);

	// Read min-max projection results from GPU
	auto minMaxResults = std::vector<float>(); minMaxResults.resize(6);
	queue->enqueueReadBuffer(minMaxBuffer, false, 0, 6 * sizeof(float), &minMaxResults[0], nullptr, &minMaxReadEvent);
	Helpers::checkErorCl(queue->finish(), "clFinish");

#ifdef TIMING_GPU
	std::cout << "TIME: Eigenvectors read: " << formatEventTime(eigenReadEvent) << std::endl;
	std::cout << "TIME: MinMax read: " << formatEventTime(minMaxReadEvent) << std::endl;
#endif

	auto t1 = Clock::Tick();
	std::cout << "TIME: Total time: " << Clock::FormatTime(t1 - t0) << std::endl;

	auto result = OOBB();
	result.center = centroid;

	result.minimums[0] = minMaxResults[0];
	result.minimums[1] = minMaxResults[1];
	result.minimums[2] = minMaxResults[2];

	result.maximums[0] = minMaxResults[3];
	result.maximums[1] = minMaxResults[4];
	result.maximums[2] = minMaxResults[5];

	result.axes[0] = glm::vec3(eigenVectors[0], eigenVectors[1], eigenVectors[2]);
	result.axes[1] = glm::vec3(eigenVectors[3], eigenVectors[4], eigenVectors[5]);
	result.axes[2] = glm::vec3(eigenVectors[6], eigenVectors[7], eigenVectors[8]);

	return result;
}

std::pair<cl::Buffer, glm::vec3> Accelerator::computeMean(std::vector<glm::vec3> &input, int workGroupSize) const
{
	auto t0 = Clock::Tick();

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

	auto t2 = Clock::Tick();
	std::cout << "TIME: Points buffer preparation: " << Clock::FormatTime(t2 - t0) << std::endl;
	//===========================================================================================
	cl_mem_flags flags = CL_MEM_READ_WRITE;
	auto dataBufferSize = data.size() * sizeof(float);
	auto resultBufferSize = groupsCount * sizeof(float);

	auto dataBuffer = cl::Buffer(*context, flags, dataBufferSize);
	auto resultBuffer = cl::Buffer(*context, flags, resultBufferSize);

	//===========================================================================================
	sumKernel->setArg(0, dataBuffer);
	sumKernel->setArg(1, workGroupSize * sizeof(float), nullptr);
	sumKernel->setArg(2, resultBuffer);
	sumKernel->setArg(3, alignedInputSize);
	sumKernel->setArg(4, alignedSize);

	cl::Event write_event;
	cl::Event sum_kernel_event;
	cl::Event reduce_kernel_event;
	cl::Event read_event;

	////===========================================================================================
	cl::NDRange local(workGroupSize);
	cl::NDRange global(alignedSize);

	queue->enqueueWriteBuffer(dataBuffer, false, 0, dataBufferSize, &data[0], 0, &write_event);
	queue->enqueueNDRangeKernel(*sumKernel, 0, global, local, nullptr, &sum_kernel_event);

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

		sumKernel->setArg(0, resultBuffer);
		sumKernel->setArg(1, workGroupSize * sizeof(float), nullptr);
		sumKernel->setArg(2, finalBuffer);
		sumKernel->setArg(3, inputSize);
		sumKernel->setArg(4, alignedSize);

		queue->enqueueNDRangeKernel(*sumKernel, 0, global, local, nullptr, &reduce_kernel_event);

		resultBuffer = finalBuffer;
		resultBufferSize = finalBufferSize;
	}

	queue->enqueueReadBuffer(resultBuffer, false, 0, resultBufferSize, &finalSum[0], nullptr, &read_event);
	Helpers::checkErorCl(queue->finish(), "clFinish");
	
	// synchronize queue
#ifdef TIMING_GPU
	std::cout << "TIME: Vertex data write: " << formatEventTime(write_event) << " Status: " << getEventStatus(write_event) << std::endl;
	std::cout << "TIME: Mean computation time: " << formatEventTime(sum_kernel_event) << " Status: " << getEventStatus(sum_kernel_event) << std::endl;
	std::cout << "TIME: Reduction time: " << (inputSize > 1 ? formatEventTime(reduce_kernel_event) : "skipped") << " Status: " << getEventStatus(reduce_kernel_event) << std::endl;
	std::cout << "TIME: Vertex data read: " << formatEventTime(read_event) << " Status: " << getEventStatus(read_event) << std::endl;
#endif
#ifdef TIMING_CPU
	auto t1 = Clock::Tick();
	std::cout << "TIME: Compute mean CPU: " << Clock::FormatTime(t1 - t0) << std::endl;
#endif

	return std::make_pair(dataBuffer, glm::vec3(finalSum[0], finalSum[1], finalSum[2]));
}

void Accelerator::centerPoints(cl::Buffer & points, int workGroupSize, int inputSize, glm::vec3 & centroid) const
{
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
	cl_float3 center = { centroid.x, centroid.y, centroid.z };

	centerKernel->setArg(0, points);
	centerKernel->setArg(1, sizeof(cl_float3), &center);

	centerKernel->setArg(2, alignedInputSize);
	centerKernel->setArg(3, alignedSize);
	centerKernel->setArg(4, inputSize);

	cl::Event kernel_event;

	//===========================================================================================
	cl::NDRange local(workGroupSize);
	cl::NDRange global(alignedSize);

	// Run kernel
	queue->enqueueNDRangeKernel(*centerKernel, 0, global, local, nullptr, &kernel_event);

#ifdef TIMING_GPU
	Helpers::checkErorCl(queue->finish(), "clFinish");
	std::cout << "TIME: Vertex centering: " << formatEventTime(kernel_event) << " Status: " << getEventStatus(kernel_event) << std::endl;
#endif
#ifdef TIMING_CPU
	auto t1 = Clock::Tick();
	std::cout << "TIME: Center points CPU: " << Clock::FormatTime(t1 - t0) << std::endl;
#endif
}

cl::Buffer Accelerator::computeCovarianceMatrix(int workGroupSize, cl::Buffer &dataBuffer, int inputSize) const
{
	auto t0 = Helpers::getTime();

	auto const pointsCount = inputSize;
	auto alignedSize = Helpers::alignSize(inputSize, workGroupSize);

	auto resultSize = Helpers::ceilDiv(inputSize, workGroupSize);
	auto nextWorkGroupSize = nextGroupSize(resultSize);
	auto nextAlignedSize = Helpers::alignSize(resultSize, nextWorkGroupSize);

	//===========================================================================================
	cl_mem_flags flags = CL_MEM_READ_WRITE;
	auto covarianceResultByteSize = 6 * nextAlignedSize * sizeof(float);
	auto covarianceResultBuffer = cl::Buffer(*context, flags, covarianceResultByteSize);
	auto covarianceSecondBuffer = cl::Buffer(*context, flags, covarianceResultByteSize);

	//===========================================================================================
	covKernel->setArg(0, dataBuffer);
	covKernel->setArg(1, covarianceResultBuffer);
	covKernel->setArg(2, workGroupSize * sizeof(float), nullptr);
	covKernel->setArg(3, inputSize);
	covKernel->setArg(4, alignedSize);
	covKernel->setArg(5, nextAlignedSize);

	cl::Event kernel_event;
	std::vector<cl::Event> reduction_events;

	//===========================================================================================
	cl::NDRange local(workGroupSize);
	cl::NDRange global(6 * alignedSize);

	// Run kernel
	queue->enqueueNDRangeKernel(*covKernel, 0, global, local, nullptr, &kernel_event);

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

		covReductionKernel->setArg(0, covarianceResultBuffer);
		covReductionKernel->setArg(1, covarianceSecondBuffer);
		covReductionKernel->setArg(2, workGroupSize * sizeof(float), nullptr);
		covReductionKernel->setArg(3, inputSize);
		covReductionKernel->setArg(4, alignedSize);
		covReductionKernel->setArg(5, nextAlignedSize);
		covReductionKernel->setArg(6, multiplier);

		cl::Event *reduction_event = nullptr;
#ifdef TIMING_GPU
		reduction_events.push_back(cl::Event());
		reduction_event = static_cast<cl::Event*>(&reduction_events.back());
#endif

		queue->enqueueNDRangeKernel(*covReductionKernel, 0, global, local, nullptr, reduction_event);

		Swap(covarianceSecondBuffer, covarianceResultBuffer);
	}

#ifdef TIMING_GPU
	Helpers::checkErorCl(queue->finish(), "clFinish");
	std::cout << "TIME: First covariance pass: " << formatEventTime(kernel_event) << " Status: " << getEventStatus(kernel_event) << std::endl;
	std::cout << "TIME: Covariance reductions: " << Clock::FormatTime(getEventVectorTime(reduction_events)) << std::endl;
#endif
#ifdef TIMING_CPU
	auto t1 = Clock::Tick();
	std::cout << "TIME: Covariance CPU: " << Clock::FormatTime(t1 - t0) << std::endl;
#endif

	return covarianceResultBuffer;
}

cl::Buffer Accelerator::computeEigenVector(cl::Buffer &covarianceMatrix) const
{
	auto t0 = Helpers::getTime();

	auto result = std::vector<float>(9);

	//===========================================================================================
	cl_mem_flags flags = CL_MEM_READ_WRITE;

	auto eigensBufferSize = 9 * sizeof(float);
	auto eigensBuffer = cl::Buffer(*context, flags, eigensBufferSize);

	//===========================================================================================
	eigenKernel->setArg(0, covarianceMatrix);
	eigenKernel->setArg(1, eigensBuffer);

	// compute results on host
	cl::Event kernel_event;

	//===========================================================================================
	cl::NDRange local(1);
	cl::NDRange global(1);

	// Run kernel
	queue->enqueueNDRangeKernel(*eigenKernel, 0, global, local, nullptr, &kernel_event);

#ifdef TIMING_GPU
	Helpers::checkErorCl(queue->finish(), "clFinish");
	std::cout << "TIME: Eigen vectors: " << formatEventTime(kernel_event) << " Status: " << getEventStatus(kernel_event) << std::endl;
#endif
#ifdef TIMING_CPU
	auto t1 = Clock::Tick();
	std::cout << "TIME: Eigens CPU: " << Clock::FormatTime(t1 - t0) << std::endl;
#endif

	return eigensBuffer;
}

cl::Buffer Accelerator::computeMinMax(cl::Buffer &points, cl::Buffer &eigens, int inputSize, int workGroupSize) const
{
	auto t0 = Helpers::getTime();

	auto alignedSize = Helpers::alignSize(inputSize, workGroupSize);
	if (alignedSize > threadCount)
	{
		// We have to be sure its dividable by 3
		alignedSize = Helpers::alignSize(threadCount, workGroupSize);
	}

	int globalCount = alignedSize;
	int groupsCount = alignedSize / workGroupSize;

	//===========================================================================================
	cl_mem_flags flags = CL_MEM_READ_WRITE;
	auto resultBufferSize = 6 * groupsCount * sizeof(float);
	auto resultBuffer = cl::Buffer(*context, flags, resultBufferSize);
	auto resultBufferSwap = cl::Buffer(*context, flags, resultBufferSize / groupsCount);

	//===========================================================================================
	projKernel->setArg(0, points);
	projKernel->setArg(1, resultBuffer);
	projKernel->setArg(2, eigens);
	projKernel->setArg(3, 9 * sizeof(float), nullptr);
	projKernel->setArg(4, 6 * workGroupSize * sizeof(float), nullptr);
	projKernel->setArg(5, inputSize);
	projKernel->setArg(6, alignedSize);
	projKernel->setArg(7, Helpers::alignSize(inputSize, workGroupSize));

	// compute results on host
	cl::Event kernel_event;

	//===========================================================================================
	cl::NDRange local(workGroupSize);
	cl::NDRange global(alignedSize);

	// Run kernel
	queue->enqueueNDRangeKernel(*projKernel, 0, global, local, nullptr, &kernel_event);

	inputSize = groupsCount;

	int count = 0;
	while (groupsCount > 1)
	{
		alignedSize = Helpers::alignSize(inputSize, workGroupSize);
		globalCount = alignedSize;
		groupsCount = alignedSize / workGroupSize;

		projReductionKernel->setArg(0, resultBuffer);
		projReductionKernel->setArg(1, resultBufferSwap);
		projReductionKernel->setArg(2, 6 * workGroupSize * sizeof(float), nullptr);
		projReductionKernel->setArg(3, inputSize);
		projReductionKernel->setArg(4, alignedSize);

		queue->enqueueNDRangeKernel(*projReductionKernel, 0, cl::NDRange(alignedSize), cl::NDRange(workGroupSize), nullptr, &kernel_event);

		inputSize = groupsCount;
		Swap(resultBuffer, resultBufferSwap);
		count++;
	}

	std::cout << "POCET CYKLU PROJECTION: " << count << std::endl;
#ifdef TIMING_GPU
	Helpers::checkErorCl(queue->finish(), "clFinish");
	std::cout << "TIME: Min/Max projection: " << formatEventTime(kernel_event) << " Status: " << getEventStatus(kernel_event) << std::endl;
#endif
#ifdef TIMING_CPU
	auto t1 = Clock::Tick();
	std::cout << "TIME: Projection CPU: " << Clock::FormatTime(t1 - t0) << std::endl;
#endif

	return resultBuffer;
}

__inline cl_float4 vec3_to_float4(glm::vec3ext v)
{
	cl_float4 c;
	c.x = v.x;
	c.y = v.y;
	c.z = v.z;
	c.w = 0.f;
	return c;
}

bool compareVecs(glm::vec3 a, cl_float4 b)
{
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

int findIndexOf(std::vector<glm::vec3ext> normals, cl_float4 f)
{
	for (auto i = 0; i < normals.size(); i++)
		if (compareVecs(normals[i], f))
			return i;

	return -1;
}

std::vector<std::vector<int>> Accelerator::sidepodals(std::vector<glm::vec3ext> normals, std::vector<std::pair<int, int>> facesForEdge, int workGroupSize) const
{
	auto t0 = Helpers::getTime();

	std::vector<cl_float4> edgeNormals;
	edgeNormals.resize(facesForEdge.size() * 2);

	for (auto i = 0; i < facesForEdge.size(); i++)
	{
		auto faces = facesForEdge[i];
		edgeNormals[i * 2] = vec3_to_float4(normals[faces.first]);
		edgeNormals[i * 2 + 1] = vec3_to_float4(normals[faces.second]);
	}

	auto inputSize = facesForEdge.size();
	auto globalCount = Helpers::alignSize(inputSize, workGroupSize);
	// Align data so that threads may access memory without checks
	for (auto i = inputSize; i < globalCount; i++) {
		// Push 2 normals for each missing edge
		edgeNormals.push_back(cl_float4{ 0,0,0,0 });
		edgeNormals.push_back(cl_float4{ 0,0,0,0 });
	}

	auto t1 = Helpers::getTime();

	std::vector<unsigned char> results;
	results.resize(inputSize * inputSize);

	//===========================================================================================
	cl_mem_flags flags = CL_MEM_READ_WRITE;
	auto dataBufferSize = edgeNormals.size() * sizeof(cl_float4);
	auto resultBufferSize = results.size() * sizeof(cl_uchar);

	auto dataBuffer = cl::Buffer(*context, flags, dataBufferSize);
	auto resultBuffer = cl::Buffer(*context, flags, resultBufferSize);

	//===========================================================================================
	sidepodalKernel->setArg(0, dataBuffer);
	sidepodalKernel->setArg(1, resultBuffer);
	sidepodalKernel->setArg(2, workGroupSize * 4 * sizeof(cl_float4), nullptr);
	sidepodalKernel->setArg(3, inputSize);

	cl::Event write_event;
	cl::Event kernel_event;
	cl::Event read_event;

	//===========================================================================================
	cl::NDRange local(workGroupSize, workGroupSize);
	cl::NDRange global(globalCount, globalCount);

	// Write data
	Helpers::checkErorCl(queue->enqueueWriteBuffer(dataBuffer, false, 0, dataBufferSize, &edgeNormals[0], nullptr, &write_event), 
		"Sidepodal write");

	// Run kernel
	Helpers::checkErorCl(queue->enqueueNDRangeKernel(*sidepodalKernel, 0, global, local, nullptr, &kernel_event), 
		"Sidepodal kernel");

	// Read results
	Helpers::checkErorCl(queue->enqueueReadBuffer(resultBuffer, false, 0, resultBufferSize, &results[0], nullptr, &read_event), 
		"Sidepodal read");

	Helpers::checkErorCl(queue->finish(), "clFinish");

#ifdef TIMING_GPU
	std::cout << "TIME: Sidepodals write: " << formatEventTime(write_event) << " Status: " << getEventStatus(write_event) << std::endl;
	std::cout << "TIME: Sidepodals kernel: " << formatEventTime(kernel_event) << " Status: " << getEventStatus(kernel_event) << std::endl;
	std::cout << "TIME: Sidepodals read: " << formatEventTime(read_event) << " Status: " << getEventStatus(read_event) << std::endl;
#endif
#ifdef TIMING_CPU
	auto t2 = Clock::Tick();
	std::cout << "TIME: Sidepodals data prep: " << Clock::FormatTime(t1 - t0) << std::endl;
	std::cout << "TIME: Sidepodals total CPU: " << Clock::FormatTime(t2 - t0) << std::endl;
#endif

	std::vector<std::vector<int>> compatibleEdges;
	compatibleEdges.resize(facesForEdge.size());
	for (auto i = 0; i < inputSize; i++)
	{
		for (auto j = i; j < inputSize; j++)
		{
			if (results[i * inputSize + j])
			{
				compatibleEdges[i].push_back(j);

				if (i != j)
					compatibleEdges[j].push_back(i);
			}
		}
	}

	return compatibleEdges;
}