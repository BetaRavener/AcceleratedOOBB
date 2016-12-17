#pragma once
#ifndef CLOCK_H
#define CLOCK_H
#include "src/Helpers.h"
#include <sstream>

typedef double tick_t;

class Clock
{
public:
	static tick_t Tick() { return Helpers::getTime(); }
	static float TimespanToMillisecondsF(tick_t a, tick_t b) { return (b-a) * 1000; }
	static std::string FormatTime(double t, bool milliseconds = true)
	{
		std::ostringstream oss;
		if (milliseconds)
			oss << t * 1000 << " ms";
		else
			oss << t << " s";

		return oss.str();
	}
};



#endif