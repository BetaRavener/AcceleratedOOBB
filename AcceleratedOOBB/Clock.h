#pragma once
#ifndef CLOCK_H
#define CLOCK_H
#include "src/Helpers.h"

typedef double tick_t;

class Clock
{
public:
	static tick_t Tick() { return Helpers::getTime(); }
	static float TimespanToMillisecondsF(tick_t a, tick_t b) { return (b-a) * 1000; }
};



#endif