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
	OOBB CreateOOBB(std::vector<glm::vec3> & points);
	static glm::vec3 ComputeCentroid(std::vector<glm::vec3> & points);
	static glm::mat3x3 ComputeCovarianceMatrix(std::vector<glm::vec3> & points);
	std::vector<glm::vec3> CreateEigens(std::vector<glm::vec3> & points);

private:
	glm::vec3 ComputeEigenValues(glm::mat3x3 covariance);
	std::vector<glm::vec3> Cpu::ComputeEigenVectors(glm::mat3x3 covariance, glm::vec3 eigv);

};
#endif CPU_H
