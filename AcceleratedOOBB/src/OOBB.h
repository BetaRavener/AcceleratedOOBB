#pragma once
#ifndef OOBB_H
#define OOBB_H
#include <glm/detail/type_vec3.hpp>
#include <string>

class OOBB
{
public:
	glm::vec3 center;
	glm::vec3 axes[3];
	float minimums[3]; 
	float maximums[3];
	OOBB();
	~OOBB();

	friend std::ostream& operator<<(std::ostream&, const OOBB&);
};

#endif // !OOBB_H


