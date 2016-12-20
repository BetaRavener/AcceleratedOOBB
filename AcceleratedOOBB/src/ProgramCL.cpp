#include "ProgramCL.h"
#include <fstream>
#include "Helpers.h"
#include <iostream>

ProgramCL::ProgramCL(cl::Device device, cl::Context context, std::vector<std::string> sources)
{
	_device = device;
	_context = context;

	cl::Program::Sources clSources;
	std::vector<std::string> contents;
	for (auto fileName : sources)
	{
		// Preserve content until program is build so that c_str pointer is still valid
		contents.push_back(readSource(fileName));
		clSources.push_back(std::pair<const char *, size_t>(contents.back().c_str(), contents.back().size()));
	}

	cl_int codes[2];
	_program = cl::Program(_context, clSources, codes);
	Helpers::checkErorCl(codes[0], "clProgram");

	// build program
	if ((codes[0] = _program.build(std::vector<cl::Device>(1, _device), "", nullptr, nullptr)) == CL_BUILD_PROGRAM_FAILURE)
	{
		std::cerr << "Build log:\n %s" << _program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(_device, &codes[1]).c_str();
		Helpers::checkErorCl(codes[1], "cl::Program::getBuildInfo<CL_PROGRAM_BUILD_LOG>");
	}
	Helpers::checkErorCl(codes[0], "clBuildProgram");
}

cl::Kernel ProgramCL::getKernel(std::string entryPointName) const
{
	cl_int code;
	cl::Kernel kernel(_program, entryPointName.c_str(), &code);
	Helpers::checkErorCl(code, "cl::Kernel");
	return kernel;
}

std::string ProgramCL::readSource(std::string fileName)
{
	auto fin = std::ifstream(fileName, std::ifstream::in | std::ifstream::binary);
	if (fin.bad() || fin.fail())
		throw std::exception("Couldn't open source file");

	std::string content;

	fin.seekg(0, std::ios::end);
	content.reserve(fin.tellg());
	fin.seekg(0, std::ios::beg);

	content.assign((std::istreambuf_iterator<char>(fin)),
		std::istreambuf_iterator<char>());
	
	return content;
}
