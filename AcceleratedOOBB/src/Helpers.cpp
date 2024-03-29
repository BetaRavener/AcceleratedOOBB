#include "Helpers.h"
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <CL/cl.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif //WIN32

#define CONTROL_OUTPUT false

void Helpers::clearConsole()
{
#ifdef _WIN32
		std::system("cls");
#else
		// Assume POSIX
		std::system("clear");
#endif
}

int Helpers::alignSize(int size, int alignTo)
{
	return ((size - 1 + alignTo) / alignTo) * alignTo;
}

int Helpers::ceilDiv(int n, int divisor)
{
	return ((n - 1 + divisor) / divisor);
}

std::string Helpers::toLower(std::string s)
{
	std::string ret;
	ret.reserve(s.size());
	for (auto c : s)
		ret += tolower(c);
	
	return ret;
}

void Helpers::checkErorCl(int code, std::string description)
{
	if (code != CL_SUCCESS && code != CL_DEVICE_NOT_FOUND) {
		std::cerr << Helpers::getTime << " ERROR: " << description << " (" << code << ", " << Helpers::resolveErrorCl(code) << ")" << std::endl;
		throw std::exception("" + code);
	}
	
	if (CONTROL_OUTPUT) {
		std::cout << Helpers::getTime() << " OK: " << description << std::endl;
	}
}

double Helpers::getTime()
{
#if _WIN32 // WINDOWS
		static auto initialized = 0;
		static LARGE_INTEGER frequency;
		LARGE_INTEGER value;

		if (!initialized) {
			initialized = 1;
			if (QueryPerformanceFrequency(&frequency) == 0) { // Hi-Res counter not available
				exit(-1);
			}
		}

		QueryPerformanceCounter(&value);
		return static_cast<double>(value.QuadPart) / static_cast<double>(frequency.QuadPart);

#else // UNIX
		struct timeval tv;
		if (gettimeofday(&tv, NULL) == -1) 
			exit(-2);
		}
		return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.;
#endif
}

std::string Helpers::resolveErrorCl(int error)
{
	switch (error) {
		case CL_SUCCESS:
			return "Success!";
		case CL_DEVICE_NOT_FOUND:
			return "Device not found.";
		case CL_DEVICE_NOT_AVAILABLE:
			return "Device not available";
		case CL_COMPILER_NOT_AVAILABLE:
			return "Compiler not available";
		case CL_MEM_OBJECT_ALLOCATION_FAILURE:
			return "Memory object allocation failure";
		case CL_OUT_OF_RESOURCES:
			return "Out of resources";
		case CL_OUT_OF_HOST_MEMORY:
			return "Out of host memory";
		case CL_PROFILING_INFO_NOT_AVAILABLE:
			return "Profiling information not available";
		case CL_MEM_COPY_OVERLAP:
			return "Memory copy overlap";
		case CL_IMAGE_FORMAT_MISMATCH:
			return "Image format mismatch";
		case CL_IMAGE_FORMAT_NOT_SUPPORTED:
			return "Image format not supported";
		case CL_BUILD_PROGRAM_FAILURE:
			return "Program build failure";
		case CL_MAP_FAILURE:
			return "Map failure";
		case CL_INVALID_VALUE:
			return "Invalid value";
		case CL_INVALID_DEVICE_TYPE:
			return "Invalid device type";
		case CL_INVALID_PLATFORM:
			return "Invalid platform";
		case CL_INVALID_DEVICE:
			return "Invalid device";
		case CL_INVALID_CONTEXT:
			return "Invalid context";
		case CL_INVALID_QUEUE_PROPERTIES:
			return "Invalid queue properties";
		case CL_INVALID_COMMAND_QUEUE:
			return "Invalid command queue";
		case CL_INVALID_HOST_PTR:
			return "Invalid host pointer";
		case CL_INVALID_MEM_OBJECT:
			return "Invalid memory object";
		case CL_INVALID_IMAGE_FORMAT_DESCRIPTOR:
			return "Invalid image format descriptor";
		case CL_INVALID_IMAGE_SIZE:
			return "Invalid image size";
		case CL_INVALID_SAMPLER:
			return "Invalid sampler";
		case CL_INVALID_BINARY:
			return "Invalid binary";
		case CL_INVALID_BUILD_OPTIONS:
			return "Invalid build options";
		case CL_INVALID_PROGRAM:
			return "Invalid program";
		case CL_INVALID_PROGRAM_EXECUTABLE:
			return "Invalid program executable";
		case CL_INVALID_KERNEL_NAME:
			return "Invalid kernel name";
		case CL_INVALID_KERNEL_DEFINITION:
			return "Invalid kernel definition";
		case CL_INVALID_KERNEL:
			return "Invalid kernel";
		case CL_INVALID_ARG_INDEX:
			return "Invalid argument index";
		case CL_INVALID_ARG_VALUE:
			return "Invalid argument value";
		case CL_INVALID_ARG_SIZE:
			return "Invalid argument size";
		case CL_INVALID_KERNEL_ARGS:
			return "Invalid kernel arguments";
		case CL_INVALID_WORK_DIMENSION:
			return "Invalid work dimension";
		case CL_INVALID_WORK_GROUP_SIZE:
			return "Invalid work group size";
		case CL_INVALID_WORK_ITEM_SIZE:
			return "Invalid work item size";
		case CL_INVALID_GLOBAL_OFFSET:
			return "Invalid global offset";
		case CL_INVALID_EVENT_WAIT_LIST:
			return "Invalid event wait list";
		case CL_INVALID_EVENT:
			return "Invalid event";
		case CL_INVALID_OPERATION:
			return "Invalid operation";
		case CL_INVALID_GL_OBJECT:
			return "Invalid OpenGL object";
		case CL_INVALID_BUFFER_SIZE:
			return "Invalid buffer size";
		case CL_INVALID_MIP_LEVEL:
			return "Invalid mip-map level";
		default:
			return "Unknown";
	}
}
