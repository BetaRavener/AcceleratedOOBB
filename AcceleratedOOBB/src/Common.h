#pragma once
#ifndef COMMON_H
#define COMMON_H

template<typename T> inline void Swap(T &a, T &b)
{
	T temp = a;
	a = b;
	b = temp;
}

#endif