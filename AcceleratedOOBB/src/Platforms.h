#pragma once
#ifndef PLATFORMS_H
#define PLATFORMS_H

#include <CL/cl.hpp>

class Platforms
{
public:
	static void printAllInfos();
	static void printDeviceInfo(cl::Device device);
	static std::string deviceTypeToString(cl_uint type);
	static cl_uint getDeviceType(cl::Device device);
	static cl::Device getCpu();
	static cl::Device getGpu(std::string vendorLowercase = "");
	static cl::Device findPowerfulGpu();
};

#endif