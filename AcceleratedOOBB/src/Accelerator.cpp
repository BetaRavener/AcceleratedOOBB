#include "Accelerator.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include <CL/cl.hpp>
#include "ProgramCL.h"
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

void Accelerator::run2(std::vector<glm::vec3> &input, int workGroupSize)
{
	cl_int code;

	auto inputSize = input.size();
	const auto alignedSize = Helpers::alignSize(inputSize, workGroupSize);

	auto data = std::vector<float>();
	data.resize(3 * alignedSize);

	for (auto i = 0; i < 3; i++)
		for (auto j = 0; j < alignedSize; j++)
			data[i*alignedSize + j] = j < inputSize ? input[j][i] : 0;

	// Just to check intermediate values
	auto covIntermResult = std::vector<float>();
	covIntermResult.resize(6 * alignedSize / workGroupSize);

	Platforms::printAllInfos();

	//===========================================================================================
	/* ======================================================
	* TODO 1. Cast
	* ziskat gpu device
	* =======================================================
	*/
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
	/* ======================================================
	* TODO 2. Cast
	* vytvorit context a query se zapnutym profilovanim
	* =======================================================
	*/
	auto context = cl::Context(gpu_device);;
	auto queue = cl::CommandQueue(context, gpu_device, CL_QUEUE_PROFILING_ENABLE);

	auto program = ProgramCL(gpu_device, context, { "../Kernels/covariance_mat.cl" });
	auto kernel = program.getKernel("covariance_matrix");

	//===========================================================================================
	/* ======================================================
	* TODO 3. Cast
	* vytvorit buffery
	* =======================================================
	*/
	cl_mem_flags flags = CL_MEM_READ_WRITE;
	auto dataBufferSize = data.size() * sizeof(float);
	auto covIntermBufferSize = 6 * alignedSize / workGroupSize * sizeof(float);
	auto dataBuffer = cl::Buffer(context, flags, dataBufferSize);
	auto covarianceIntermediateBuffer = cl::Buffer(context, flags, covIntermBufferSize);

	//===========================================================================================
	/* ======================================================
	* TODO 4. Cast
	* nastavit parametry spusteni
	* =======================================================
	*/

	kernel.setArg(0, dataBuffer);
	kernel.setArg(1, covarianceIntermediateBuffer);
	kernel.setArg(2, workGroupSize, nullptr);
	kernel.setArg(3, inputSize);
	kernel.setArg(4, alignedSize);

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
	/* ======================================================
	* TODO 5. Cast
	* velikost skupiny, kopirovat data na gpu, spusteni kernelu, kopirovani dat zpet
	* pro zarovnání muzete pouzit funkci iCeilTo(co, na_nasobek_ceho)
	* jako vystupni event kopirovani nastavte prepripravene eventy a_event b_event c_event
	* vystupni event kernelu kernel_event
	* =======================================================
	*/

	cl::NDRange local(workGroupSize);
	cl::NDRange global(6 * alignedSize);

	// Write data to GPU
	queue.enqueueWriteBuffer(dataBuffer, false, 0, dataBufferSize, &data[0], nullptr, &a_event);

	// Run kernel
	queue.enqueueNDRangeKernel(kernel, 0, global, local, nullptr, &kernel_event);

	// Read data from GPU
	queue.enqueueReadBuffer(covarianceIntermediateBuffer, false, 0, covIntermBufferSize, &covIntermResult[0], nullptr, &c_event);

	// synchronize queue
	Helpers::checkErorCl(queue.finish(), "clFinish");

	float check = 0;
	for (auto i = 0; i < alignedSize / workGroupSize; i++)
		check += covIntermResult[i];
	check /= inputSize;

	auto cov = Cpu::ComputeCovarianceMatrix(input, glm::vec3(0, 0, 0));

	auto t2 = Helpers::getTime();

	return;
}


void Accelerator::run3(std::vector<glm::vec3> &input, std::vector<glm::vec3> eigens, int workGroupSize)
{
	cl_int code;

	auto inputSize = input.size();
	const auto alignedSize = Helpers::alignSize(inputSize, workGroupSize);
	int globalCount = alignedSize;
	int groupsCount = alignedSize / workGroupSize;

	auto points = std::vector<float>();
	points.resize(3 * alignedSize);

	for (auto i = 0; i < 3; i++)
		for (auto j = 0; j < alignedSize; j++)
			points[i*alignedSize + j] = j < inputSize ? input[j][i] : 0;

	auto eigenVector = std::vector<float>(9);

	for (auto i = 0; i < 3; i++)
	{
		eigenVector[3 * i + 0] = eigens[i].x;
		eigenVector[3 * i + 1] = eigens[i].y;
		eigenVector[3 * i + 2] = eigens[i].z;
	}

	// Just to check intermediate values
	auto minMaxResults = std::vector<float>();
	minMaxResults.resize(6 * groupsCount);

	Platforms::printAllInfos();

	//===========================================================================================
	/* ======================================================
	* TODO 1. Cast
	* ziskat gpu device
	* =======================================================
	*/
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
	/* ======================================================
	* TODO 2. Cast
	* vytvorit context a query se zapnutym profilovanim
	* =======================================================
	*/
	auto context = cl::Context(gpu_device);;
	auto queue = cl::CommandQueue(context, gpu_device, CL_QUEUE_PROFILING_ENABLE);

	auto program = ProgramCL(gpu_device, context, { "../Kernels/projection_mat.cl" });
	auto kernel = program.getKernel("projection_matrix");

	//===========================================================================================
	/* ======================================================
	* TODO 3. Cast
	* vytvorit buffery
	* =======================================================
	*/

	cl_mem_flags flags = CL_MEM_READ_WRITE;
	auto dataBufferSize = points.size() * sizeof(float);
	auto dataBuffer = cl::Buffer(context, flags, dataBufferSize);

	auto resultBufferSize = 6 * groupsCount * sizeof(float);
	auto resultBuffer = cl::Buffer(context, flags, resultBufferSize);

	auto eigensBufferSize = 9 * sizeof(float);
	auto eigensBuffer = cl::Buffer(context, flags, eigensBufferSize);
	//===========================================================================================
	/* ======================================================
	* TODO 4. Cast
	* nastavit parametry spusteni
	* =======================================================
	*/

	kernel.setArg(0, dataBuffer);
	kernel.setArg(1, resultBuffer);
	kernel.setArg(2, eigensBuffer);
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
	/* ======================================================
	* TODO 5. Cast
	* velikost skupiny, kopirovat data na gpu, spusteni kernelu, kopirovani dat zpet
	* pro zarovnání muzete pouzit funkci iCeilTo(co, na_nasobek_ceho)
	* jako vystupni event kopirovani nastavte prepripravene eventy a_event b_event c_event
	* vystupni event kernelu kernel_event
	* =======================================================
	*/

	cl::NDRange local(workGroupSize);
	cl::NDRange global(alignedSize);

	// Write data to GPU
	queue.enqueueWriteBuffer(dataBuffer, false, 0, dataBufferSize, &points[0], nullptr, &a_event);
	queue.enqueueWriteBuffer(eigensBuffer, false, 0, eigensBufferSize, &eigenVector[0], nullptr, &a_event);


	// Run kernel
	queue.enqueueNDRangeKernel(kernel, 0, global, local, nullptr, &kernel_event);

	// Read data from GPU
	queue.enqueueReadBuffer(resultBuffer, false, 0, resultBufferSize, &minMaxResults[0], nullptr, &c_event);

	// synchronize queue
	Helpers::checkErorCl(queue.finish(), "clFinish");

	auto min = glm::vec3(0, 0, 0);
	auto max = glm::vec3(0, 0, 0);
	// Min
	for (auto i = 0; i < minMaxResults.size() / 2; i += 3)
	{
		if (minMaxResults[i] < min.x)
			min.x = minMaxResults[i];
		if (minMaxResults[i + 1] < min.y)
			min.y = minMaxResults[i + 1];
		if (minMaxResults[i + 2]  < min.z)
			min.z = minMaxResults[i + 2];
	}

	for (auto i = minMaxResults.size() / 2; i < minMaxResults.size(); i += 3)
	{
		if (minMaxResults[i] > max.x)
			max.x = minMaxResults[i];
		if (minMaxResults[i + 1] > max.y)
			max.y = minMaxResults[i + 1];
		if (minMaxResults[i + 2] > max.z)
			max.z = minMaxResults[i + 2];
	}

 	auto t2 = Helpers::getTime();

	return;
}