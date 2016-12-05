#pragma once
#ifndef ACCELEREATOR_H
#define ACCELEREATOR_H

class Accelerator
{
public:
	void run();
private:
	static unsigned int alignCount(int dataSize, int alignSize);
};

#endif