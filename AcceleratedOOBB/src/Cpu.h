#pragma once
#ifndef CPU_H
#define CPU_H

#include <glm/detail/type_vec3.hpp>
#include <glm/detail/type_mat3x3.hpp>
#include <glm/matrix.hpp>
#include <vector>
#include "OOBB.h"

class Cpu
{
public:
	Cpu();
	~Cpu();

	OOBB CreateOOBB(std::vector<glm::vec3> & points);

private:
	glm::vec3 ComputeCentroid(std::vector<glm::vec3> & points);
	glm::dmat3x3 ComputeCovarianceMatrix(std::vector<glm::vec3> & points, glm::vec3 & centroid);
	glm::vec3 ComputeEigenValues(glm::mat3x3 covariance);

};
#endif CPU_H
