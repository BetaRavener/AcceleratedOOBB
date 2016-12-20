#pragma once
#ifndef HELPERS_H
#define HELPERS_H
#include <string>

class Helpers
{
public:
	static void clearConsole();
	static int alignSize(int size, int alignTo);
	static int ceilDiv(int n, int divisor);
	static std::string toLower(std::string s);
	static void checkErorCl(int code, std::string description);
	static double getTime();
	static std::string resolveErrorCl(int error);
};

#endif