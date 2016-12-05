#pragma once
#ifndef PROGRAM_CL_H
#define PROGRAM_CL_H
#include <string>
#include <vector>
#include <CL/cl.hpp>

class ProgramCL
{
public:
	ProgramCL(cl::Device device, cl::Context context, std::vector<std::string> sources);
	cl::Kernel getKernel(std::string entryPointName) const;
private:
	static std::string readSource(std::string fileName);

	cl::Device _device;
	cl::Context _context;
	cl::Program _program;
};

#endif