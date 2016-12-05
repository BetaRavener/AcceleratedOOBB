#pragma once
#ifndef HELPERS_H
#define HELPERS_H
#include <string>
#include <CL/cl.hpp>

class Helpers
{
public:
	static std::string toLower(std::string s);
	static void checkErorCl(cl_int code, std::string description);
	static double getTime();
	static std::string resolveErrorCl(cl_int error);
};

#endif