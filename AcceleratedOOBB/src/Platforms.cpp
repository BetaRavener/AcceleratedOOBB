#include "Platforms.h"
#include "Helpers.h"
#include <iostream>
#include <algorithm>

//#define INTEL_EXPERIMENTAL

void Platforms::printAllInfos()
{
	int code;
	std::vector<cl::Platform> platforms;
	std::vector<cl::Device> platform_devices;

	// Get Platforms count
	Helpers::checkErorCl(cl::Platform::get(&platforms), "cl::Platform::get");
	std::cout << "Platforms: " << std::endl;
	for (auto i = 0; i < platforms.size(); i++)
	{
		// Print platform name
		auto pName = platforms[i].getInfo<CL_PLATFORM_NAME>(&code);
		Helpers::checkErorCl(code, "cl::Platform::getInfo<CL_PLATFORM_NAME>");
		std::cout << i << " . Platform name: " << pName.c_str() << std::endl;

		// Get platform devices count
		Helpers::checkErorCl(platforms[i].getDevices(CL_DEVICE_TYPE_ALL, &platform_devices), "getDevices");
		if (platform_devices.empty()) {
			std::cout << "  No devices" << std::endl;
			continue;
		}

		for (auto j = 0; j < platform_devices.size(); j++)
		{
			// Get device name
			auto dName = platform_devices[j].getInfo<CL_DEVICE_NAME>(&code);
			Helpers::checkErorCl(code, "cl::Device::getInfo<CL_DEVICE_NAME>");
			std::cout << "  " << j << ". Device name: " << dName.c_str() << std::endl;
		}
		platform_devices.clear();
	}
}

void Platforms::printDeviceInfo(cl::Device device)
{
	cl_int code;
	auto dType = getDeviceType(device);
	auto dName = device.getInfo<CL_DEVICE_NAME>(&code);
	Helpers::checkErorCl(code, "cl::Device::getInfo<CL_DEVICE_NAME>");
	std::cout << deviceTypeToString(dType) << " - "<< dName.c_str();
}

std::string Platforms::deviceTypeToString(cl_uint type)
{
	switch(type)
	{
	case CL_DEVICE_TYPE_CPU:
		return "CPU";
	case CL_DEVICE_TYPE_GPU:
		return "GPU";
	case CL_DEVICE_TYPE_ACCELERATOR:
		return "Accelerator";
	case CL_DEVICE_TYPE_CUSTOM:
		return "Custom";
	case CL_DEVICE_TYPE_DEFAULT:
	case CL_DEVICE_TYPE_ALL:
		return "Enum";
	default:
		return "Unknown type";
	}
}

cl_uint Platforms::getDeviceType(cl::Device device)
{
	cl_int code;
	auto dType = device.getInfo<CL_DEVICE_TYPE>(&code);
	Helpers::checkErorCl(code, "cl::Device::getInfo<CL_DEVICE_TYPE>");
	return dType;
}

cl::Device Platforms::getCpu()
{
	cl_int code;
	std::vector<cl::Platform> platforms;
	std::vector<cl::Device> platform_devices;

	// Get Platforms count
	Helpers::checkErorCl(cl::Platform::get(&platforms), "cl::Platform::get");
	for (auto platform : platforms)
	{
		auto pName = platform.getInfo<CL_PLATFORM_NAME>(&code);
		Helpers::checkErorCl(code, "cl::Platform::getInfo<CL_PLATFORM_NAME>");
		auto pNameLower = Helpers::toLower(pName);

#ifdef INTEL_EXPERIMENTAL
		if (pNameLower.find("intel") != std::string::npos && pNameLower.find("experimental") == std::string::npos)
			continue;
#endif

		// Get platform devices count
		Helpers::checkErorCl(platform.getDevices(CL_DEVICE_TYPE_CPU, &platform_devices), "getDevices");
		if (platform_devices.empty())
			continue;

		return platform_devices[0];
	}

	throw std::exception("Device not found");
}

cl::Device Platforms::getGpu(std::string vendorLowercase)
{
	cl_int code;
	std::vector<cl::Platform> platforms;
	std::vector<cl::Device> platform_devices;

	// Get Platforms count
	Helpers::checkErorCl(cl::Platform::get(&platforms), "cl::Platform::get");
	for (auto platform : platforms)
	{
		// Get platform name
		auto pName = platform.getInfo<CL_PLATFORM_NAME>(&code);
		Helpers::checkErorCl(code, "cl::Platform::getInfo<CL_PLATFORM_NAME>");
		auto pNameLower = Helpers::toLower(pName);

		// Get platform devices count
		Helpers::checkErorCl(platform.getDevices(CL_DEVICE_TYPE_GPU, &platform_devices), "getDevices");
		if (platform_devices.empty())
			continue;

		for (auto device : platform_devices)
		{
			// Get device name
			auto dName = device.getInfo<CL_DEVICE_NAME>(&code);
			Helpers::checkErorCl(code, "cl::Device::getInfo<CL_DEVICE_NAME>");

			auto dNameLower = Helpers::toLower(dName);
			if (vendorLowercase.empty() ||
				pNameLower.find(vendorLowercase) != std::string::npos ||
				dNameLower.find(vendorLowercase) != std::string::npos)
				return device;

		}
		platform_devices.clear();
	}

	throw std::exception("Device not found");
}

cl::Device Platforms::findPowerfulGpu()
{
	cl::Device gpu_device;

	// Try NVIDIA first
	try {
		gpu_device = getGpu("nvidia");
		return gpu_device;
	}
	catch (std::exception e) {
	}

	// Then AMD
	try {
		gpu_device = getGpu("amd");
		return gpu_device;
	}
	catch (std::exception e) {
	}

	// Then anything left
	gpu_device = getGpu();
	return gpu_device;
}
