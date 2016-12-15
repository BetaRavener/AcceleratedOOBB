#include "ProgramCL.h"
#include "Accelerator.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>


#include "Helpers.h"
#include "Platforms.h"
#include <iostream>
#include <glm/detail/type_vec3.hpp>
#include "Cpu.h"

#define SELECTED_DEVICE_TYPE CL_DEVICE_TYPE_CPU

#define MATRIX_W 1024
#define MATRIX_H 1024

double getEventTime(cl::Event i_event)
{
	return double(i_event.getProfilingInfo<CL_PROFILING_COMMAND_END>() - i_event.getProfilingInfo<CL_PROFILING_COMMAND_START>()) / 1000000000;
}

void Accelerator::run()
{
	cl_int code;

	// Create host buffers
	auto a_data = static_cast<cl_int *>(malloc(sizeof(cl_int) * MATRIX_W * MATRIX_H));
	auto b_data = static_cast<cl_int *>(malloc(sizeof(cl_int) * MATRIX_W * MATRIX_H));
	auto host_data = static_cast<cl_int *>(malloc(sizeof(cl_int) * MATRIX_W * MATRIX_H));
	auto device_data = static_cast<cl_int *>(malloc(sizeof(cl_int) * MATRIX_W * MATRIX_H));

	// Set input matrix data
	for (int i = 0; i < MATRIX_W * MATRIX_H; i++)
	{
		a_data[i] = i;
		b_data[i] = i;
	}

	Platforms::printAllInfos();

	//===========================================================================================
	/* ======================================================
	* TODO 1. Cast
	* ziskat gpu device
	* =======================================================
	*/
	auto gpu_device = Platforms::findPowerfulGpu();

	// check if device is correct
	if (Platforms::getDeviceType(gpu_device) == CL_DEVICE_TYPE_GPU)
	{
		std::cout << "Selected device type: Correct" << std::endl;
	}
	else
	{
		std::cout << "Selected device type: Incorrect" << std::endl;
	}
	std::cout << "Selected device: " << std::endl;
	Platforms::printDeviceInfo(gpu_device);
	std::cout << std::endl;

	//===========================================================================================
	/* ======================================================
	* TODO 2. Cast
	* vytvorit context a query se zapnutym profilovanim
	* =======================================================
	*/
	auto context = cl::Context(gpu_device);;
	auto queue = cl::CommandQueue(context, gpu_device, CL_QUEUE_PROFILING_ENABLE);

	auto program = ProgramCL(gpu_device, context, { "../Kernels/matrix_add.cl" });
	auto kernel = program.getKernel("matrix_add");

	//===========================================================================================
	/* ======================================================
	* TODO 3. Cast
	* vytvorit buffery
	* =======================================================
	*/
	cl::Buffer a_buffer;
	cl::Buffer b_buffer;
	cl::Buffer c_buffer;

	//TODO: Flags? size?
	cl_mem_flags flags = CL_MEM_READ_WRITE;
	size_t buffer_size = MATRIX_W * MATRIX_H * sizeof(cl_int);

	a_buffer = cl::Buffer(context, flags, buffer_size);
	b_buffer = cl::Buffer(context, flags, buffer_size);
	c_buffer = cl::Buffer(context, flags, buffer_size);

	cl_int matrix_width = MATRIX_W;
	cl_int matrix_height = MATRIX_H;

	//===========================================================================================
	/* ======================================================
	* TODO 4. Cast
	* nastavit parametry spusteni
	* =======================================================
	*/

	kernel.setArg(0, a_buffer);
	kernel.setArg(1, b_buffer);
	kernel.setArg(2, c_buffer);
	kernel.setArg(3, matrix_width);
	kernel.setArg(4, matrix_height);

	double t0 = Helpers::getTime();
	// compute results on host
	for (int y = 0; y < MATRIX_H; y++)
	{
		for (int x = 0; x < MATRIX_W; x++)
		{
			host_data[y * MATRIX_W + x] = a_data[y * MATRIX_W + x] + b_data[y * MATRIX_W + x];
		}
	}

	cl::UserEvent a_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent a_event");
	cl::UserEvent b_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent b_event");
	cl::UserEvent kernel_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent kernel_event");
	cl::UserEvent c_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent c_event");

	auto t1 = Helpers::getTime();

	//===========================================================================================
	/* ======================================================
	* TODO 5. Cast
	* velikost skupiny, kopirovat data na gpu, spusteni kernelu, kopirovani dat zpet
	* pro zarovnání muzete pouzit funkci iCeilTo(co, na_nasobek_ceho)
	* jako vystupni event kopirovani nastavte prepripravene eventy a_event b_event c_event
	* vystupni event kernelu kernel_event
	* =======================================================
	*/

	cl::NDRange local(16, 16);// = cl::NullRange;
	cl::NDRange global(Helpers::alignSize(MATRIX_W, 1024), Helpers::alignSize(MATRIX_H, 1024));

	// Write data to GPU
	queue.enqueueWriteBuffer(a_buffer, false, 0, buffer_size, a_data, nullptr, &a_event);
	queue.enqueueWriteBuffer(b_buffer, false, 0, buffer_size, b_data, nullptr, &b_event);

	// Run kernel
	queue.enqueueNDRangeKernel(kernel, 0, global, local, nullptr, &kernel_event);

	// Read data from GPU
	queue.enqueueReadBuffer(c_buffer, false, 0, buffer_size, device_data, nullptr, &c_event);

	// synchronize queue
	Helpers::checkErorCl(queue.finish(), "clFinish");

	auto t2 = Helpers::getTime();

	// check data
	if (memcmp(device_data, host_data, MATRIX_W * MATRIX_H * sizeof(cl_int)) == 0)
	{
		printf("\nResult: Correct\n");
	}
	else
	{
		printf("\nResult: Incorrect\n");
	}

	// print results
	printf("\nExample results:\n");
	for (auto x = 0; x < 10; x++)
	{
		auto y = x + 1;
		auto i = y * MATRIX_W + x;
		printf(" [%d,%d] %d + %d = %d(gpu) %d(cpu)\n", y, x, a_data[i], b_data[i], device_data[i], host_data[i]);
	}

	// print performance info
	printf("\nHost timers:\n");
	printf(" OpenCL processing time: %fs\n", t2 - t1);
	printf(" CPU    processing time: %fs\n", t1 - t0);
	printf("\nDevice timers:\n");
	printf(" OpenCL copy time: %fs\n", getEventTime(a_event) + getEventTime(b_event) + getEventTime(c_event));
	printf(" OpenCL processing time: %fs\n", getEventTime(kernel_event));

	// deallocate host data
	free(a_data);
	free(b_data);
	free(host_data);
	free(device_data);
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

OOBB Accelerator::mainRun(std::vector<glm::vec3> &input, int workGroupSize)
{
	Platforms::printAllInfos();

	auto gpu_device = Platforms::getCpu();

	// check if device is correct
	if (Platforms::getDeviceType(gpu_device) == CL_DEVICE_TYPE_GPU)
	{
		std::cout << "Selected device type: Correct" << std::endl;
	}
	else
	{
		std::cout << "Selected device type: Incorrect" << std::endl;
	}
	std::cout << "Selected device: " << std::endl;
	Platforms::printDeviceInfo(gpu_device);
	std::cout << std::endl;

	//===========================================================================================
	auto context = cl::Context(gpu_device);

	auto inputSize = input.size();

	cl_uint computeUnitCount;
	size_t retSize;
	Helpers::checkErorCl(
		clGetDeviceInfo(gpu_device(), CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(cl_uint), (void*)&computeUnitCount, &retSize),
		"Getting GPU INFO");

	threadCount = computeUnitCount * 2048 * 2;

	auto bufferAndSum = computeMean(input, 256, gpu_device, context);

	auto centroid = bufferAndSum.second / (float)inputSize;

	centerPoints(bufferAndSum.first, workGroupSize, gpu_device, context, inputSize, centroid);

	auto covBuffer = computeCovarianceMatrix(workGroupSize, gpu_device, context, bufferAndSum.first, inputSize);

	auto bufferAndResult = computeEigenVector(covBuffer, gpu_device, context);

	auto minMax = computeMinMax(bufferAndSum.first, bufferAndResult.first, inputSize, workGroupSize, gpu_device, context);

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

std::pair<cl::Buffer, glm::vec3> Accelerator::computeMean(std::vector<glm::vec3> &input, int workGroupSize, cl::Device &device, cl::Context &context)
{
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
	auto queue = cl::CommandQueue(context, device, CL_QUEUE_PROFILING_ENABLE);

	auto program = ProgramCL(device, context, { "../Kernels/points_sum.cl" });
	auto sumKernel = program.getKernel("points_sum");

	//===========================================================================================
	cl_mem_flags flags = CL_MEM_READ_WRITE;
	auto dataBufferSize = data.size() * sizeof(float);
	auto resultBufferSize = groupsCount * sizeof(float);

	auto dataBuffer = cl::Buffer(context, flags, dataBufferSize);
	auto resultBuffer = cl::Buffer(context, flags, resultBufferSize);

	//===========================================================================================
	sumKernel.setArg(0, dataBuffer);
	sumKernel.setArg(1, workGroupSize * sizeof(float), nullptr);
	sumKernel.setArg(2, resultBuffer);
	sumKernel.setArg(3, alignedInputSize);
	sumKernel.setArg(4, alignedSize);

	double t0 = Helpers::getTime();
	// compute results on host
	cl::UserEvent a_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent a_event");
	cl::UserEvent b_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent b_event");
	cl::UserEvent kernel_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent kernel_event");
	cl::UserEvent c_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent c_event");
	auto t1 = Helpers::getTime();

	//===========================================================================================
	cl::NDRange local(workGroupSize);
	cl::NDRange global(alignedSize);

	// Write data to GPU
	queue.enqueueWriteBuffer(dataBuffer, false, 0, dataBufferSize, &data[0], nullptr, &a_event);

	// Run kernel
	queue.enqueueNDRangeKernel(sumKernel, 0, global, local, nullptr, &kernel_event);

	// It is possible, that all is reduced, but if not, we continue reducing
	inputSize = groupsCount / 3;
	if (inputSize > 1)
	{
		alignedSize = Helpers::alignSize(inputSize, workGroupSize) * 3;
		//workGroupSize = nextWorkGroupSize;

		local = cl::NDRange(workGroupSize);
		global = cl::NDRange(alignedSize);

		auto finalBufferSize = 3 * sizeof(float);
		auto finalBuffer = cl::Buffer(context, flags, finalBufferSize);

		sumKernel.setArg(0, resultBuffer);
		sumKernel.setArg(1, workGroupSize * sizeof(float), nullptr);
		sumKernel.setArg(2, finalBuffer);
		sumKernel.setArg(3, inputSize);
		sumKernel.setArg(4, alignedSize);

		queue.enqueueNDRangeKernel(sumKernel, 0, global, local, nullptr, nullptr);

		// Read data from GPU
		queue.enqueueReadBuffer(finalBuffer, false, 0, finalBufferSize, &finalSum[0], nullptr, &c_event);
	}
	else
	{
		queue.enqueueReadBuffer(resultBuffer, false, 0, resultBufferSize, &finalSum[0], nullptr, &c_event);
	}

	// synchronize queue
	Helpers::checkErorCl(queue.finish(), "clFinish");

	auto t2 = Helpers::getTime();

	return std::make_pair(dataBuffer, glm::vec3(finalSum[0], finalSum[1], finalSum[2]));
}


void Accelerator::centerPoints(cl::Buffer & points, int workGroupSize, cl::Device &device, cl::Context &context, int inputSize, glm::vec3 & centroid)
{
	cl_int code;

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
	auto queue = cl::CommandQueue(context, device, CL_QUEUE_PROFILING_ENABLE);

	auto program = ProgramCL(device, context, { "../Kernels/center_points.cl" });
	auto centerKernel = program.getKernel("center_points");

	//===========================================================================================
	centerKernel.setArg(0, points);
	centerKernel.setArg(1, centroid.x);
	centerKernel.setArg(2, centroid.y);
	centerKernel.setArg(3, centroid.z);

	centerKernel.setArg(4, alignedInputSize);
	centerKernel.setArg(5, alignedSize);
	centerKernel.setArg(6, inputSize);

	double t0 = Helpers::getTime();

	cl::UserEvent kernel_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent kernel_event");
	cl::UserEvent c_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent c_event");
	auto t1 = Helpers::getTime();

	//===========================================================================================
	cl::NDRange local(workGroupSize);
	cl::NDRange global(alignedSize);

	// Run kernel
	queue.enqueueNDRangeKernel(centerKernel, 0, global, local, nullptr, &kernel_event);

	// synchronize queue
	Helpers::checkErorCl(queue.finish(), "clFinish");

	auto t2 = Helpers::getTime();
}


cl::Buffer Accelerator::computeCovarianceMatrix(int workGroupSize, cl::Device &device, cl::Context &context, cl::Buffer &dataBuffer, int inputSize)
{
	cl_int code;

	auto const pointsCount = inputSize;
	auto alignedSize = Helpers::alignSize(inputSize, workGroupSize);

	auto resultSize = Helpers::ceilDiv(inputSize, workGroupSize);
	auto nextWorkGroupSize = nextGroupSize(resultSize);
	auto nextAlignedSize = Helpers::alignSize(resultSize, nextWorkGroupSize);

	auto covIntermResult = std::vector<float>();
	covIntermResult.resize(6 * nextWorkGroupSize);

	
	//===========================================================================================
	auto queue = cl::CommandQueue(context, device, CL_QUEUE_PROFILING_ENABLE);

	auto program = ProgramCL(device, context, { "../Kernels/covariance_mat.cl", "../Kernels/covariance_reduce.cl" });
	auto covKernel = program.getKernel("covariance_matrix");
	auto reductionKernel = program.getKernel("cov_reduce");

	//===========================================================================================
	cl_mem_flags flags = CL_MEM_READ_WRITE;
	auto covIntermBufferSize = covIntermResult.size() * sizeof(float);
	auto covarianceIntermediateBufferA = cl::Buffer(context, flags, covIntermBufferSize);
	auto covarianceIntermediateBufferB = cl::Buffer(context, flags, covIntermBufferSize);

	//===========================================================================================
	covKernel.setArg(0, dataBuffer);
	covKernel.setArg(1, covarianceIntermediateBufferA);
	covKernel.setArg(2, workGroupSize, nullptr);
	covKernel.setArg(3, inputSize);
	covKernel.setArg(4, alignedSize);
	covKernel.setArg(5, nextAlignedSize);

	double t0 = Helpers::getTime();
	// compute results on host
	cl::UserEvent a_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent a_event");
	cl::UserEvent b_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent b_event");
	cl::UserEvent kernel_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent kernel_event");
	cl::UserEvent c_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent c_event");
	auto t1 = Helpers::getTime();

	//===========================================================================================
	cl::NDRange local(workGroupSize);
	cl::NDRange global(6 * alignedSize);

	// Run kernel
	queue.enqueueNDRangeKernel(covKernel, 0, global, local, nullptr, &kernel_event);

	//TODO: Swap buffers!!!
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

		reductionKernel.setArg(0, covarianceIntermediateBufferA);
		reductionKernel.setArg(1, covarianceIntermediateBufferB);
		reductionKernel.setArg(2, workGroupSize, nullptr);
		reductionKernel.setArg(3, inputSize);
		reductionKernel.setArg(4, alignedSize);
		reductionKernel.setArg(5, nextAlignedSize);
		reductionKernel.setArg(6, multiplier);

		queue.enqueueNDRangeKernel(reductionKernel, 0, global, local, nullptr, nullptr);
	}

	// Read data from GPU
	queue.enqueueReadBuffer(covarianceIntermediateBufferB, false, 0, covIntermBufferSize, &covIntermResult[0], nullptr, &c_event);

	// synchronize queue
	Helpers::checkErorCl(queue.finish(), "clFinish");

	auto t2 = Helpers::getTime();

	return covarianceIntermediateBufferB;
}

std::pair<cl::Buffer, std::vector<float>> Accelerator::computeEigenVector(cl::Buffer &covarianceMatrix, cl::Device &device, cl::Context &context)
{
	cl_int code;

	auto result = std::vector<float>(9);

	//===========================================================================================
	auto queue = cl::CommandQueue(context, device, CL_QUEUE_PROFILING_ENABLE);

	auto program = ProgramCL(device, context, { "../Kernels/eigenvector.cl" });
	auto kernel = program.getKernel("compute_eigens");

	//===========================================================================================
	cl_mem_flags flags = CL_MEM_READ_WRITE;

	auto eigensBufferSize = 9 * sizeof(float);
	auto eigensBuffer = cl::Buffer(context, flags, eigensBufferSize);

	//===========================================================================================
	kernel.setArg(0, covarianceMatrix);
	kernel.setArg(1, eigensBuffer);

	double t0 = Helpers::getTime();
	// compute results on host
	cl::UserEvent kernel_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent kernel_event");
	cl::UserEvent c_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent c_event");
	auto t1 = Helpers::getTime();

	//===========================================================================================
	cl::NDRange local(1);
	cl::NDRange global(1);

	// Run kernel
	queue.enqueueNDRangeKernel(kernel, 0, global, local, nullptr, &kernel_event);

	// Read data from GPU
	queue.enqueueReadBuffer(eigensBuffer, false, 0, eigensBufferSize, &result[0], nullptr, &c_event);

	// synchronize queue
	Helpers::checkErorCl(queue.finish(), "clFinish");

	auto t2 = Helpers::getTime();

	return std::make_pair(eigensBuffer, result);
}

std::pair<glm::vec3, glm::vec3> Accelerator::computeMinMax(cl::Buffer &points, cl::Buffer &eigens, int inputSize, int workGroupSize, cl::Device &device, cl::Context &context)
{
	cl_int code;

	auto alignedSize = Helpers::alignSize(inputSize, workGroupSize);
	int globalCount = alignedSize;
	int groupsCount = alignedSize / workGroupSize;

	// Just to check intermediate values
	auto minMaxResults = std::vector<float>();
	minMaxResults.resize(6 * groupsCount);


	//===========================================================================================
	auto queue = cl::CommandQueue(context, device, CL_QUEUE_PROFILING_ENABLE);

	auto program = ProgramCL(device, context, { "../Kernels/projection_mat.cl" });
	auto kernel = program.getKernel("projection_matrix");

	//auto program2 = ProgramCL(gpu_device, context, { "../Kernels/reduction_minmax.cl" });
	auto reduce_kernel = program.getKernel("reduction_minmax");

	//===========================================================================================
	cl_mem_flags flags = CL_MEM_READ_WRITE;
	auto resultBufferSize = 6 * groupsCount * sizeof(float);
	auto resultBuffer = cl::Buffer(context, flags, resultBufferSize);
	auto resultBuffer2 = cl::Buffer(context, flags, resultBufferSize);

	//===========================================================================================

	kernel.setArg(0, points);
	kernel.setArg(1, resultBuffer);
	kernel.setArg(2, eigens);
	kernel.setArg(3, 9, nullptr);
	kernel.setArg(4, 6 * workGroupSize, nullptr);
	kernel.setArg(5, inputSize);
	kernel.setArg(6, alignedSize);

	double t0 = Helpers::getTime();
	// compute results on host
	cl::UserEvent a_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent a_event");
	cl::UserEvent b_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent b_event");
	cl::UserEvent kernel_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent kernel_event");
	cl::UserEvent c_event(context, &code);
	Helpers::checkErorCl(code, "clCreateUserEvent c_event");
	auto t1 = Helpers::getTime();

	//===========================================================================================
	cl::NDRange local(workGroupSize);
	cl::NDRange global(alignedSize);

	// Run kernel
	queue.enqueueNDRangeKernel(kernel, 0, global, local, nullptr, &kernel_event);

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

		queue.enqueueNDRangeKernel(reduce_kernel, 0, cl::NDRange(alignedSize), cl::NDRange(workGroupSize), nullptr, &kernel_event);

		inputSize = groupsCount;
		cl::Buffer *tmp = inputBuffer;
		inputBuffer = result;
		result = tmp;
	}

	//// Read data from GPU
	queue.enqueueReadBuffer(*inputBuffer, false, 0, 6 * sizeof(float), &minMaxResults[0], nullptr, &c_event);

	// synchronize queue
	Helpers::checkErorCl(queue.finish(), "clFinish");
	
 	auto t2 = Helpers::getTime();

	auto min = glm::vec3(minMaxResults[0], minMaxResults[1], minMaxResults[2]);
	auto max = glm::vec3(minMaxResults[3], minMaxResults[4], minMaxResults[5]);

	return std::make_pair(min, max);
}

