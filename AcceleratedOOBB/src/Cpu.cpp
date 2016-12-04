#include "Cpu.h"



Cpu::Cpu()
{
}


Cpu::~Cpu()
{
}

OOBB Cpu::CreateOOBB(std::vector<glm::vec3> & points)
{
	auto centroid = ComputeCentroid(points);

	auto covarianceMatrix = ComputeCovarianceMatrix(points, centroid);


	return OOBB();
}

glm::vec3 Cpu::ComputeCentroid(std::vector<glm::vec3> & points)
{
	auto centroid = glm::vec3(0, 0, 0);
	for (unsigned int i = 0; i < points.size(); i++)
	{
		centroid += points[i];
	}

	centroid /= points.size();

	return centroid;
}

float** createMatrix(int rowCount, int colCount)
{
	float** ary = new float*[rowCount];
	for (int i = 0; i < rowCount; ++i)
		ary[i] = new float[colCount];

	return ary;
}

glm::mat3x3 Cpu::ComputeCovarianceMatrix(std::vector<glm::vec3> & points, glm::vec3 & centroid)
{
	std::vector<glm::vec3> centeredPoints(points.size());
	
	float** matrix = new float*[3];
	for (unsigned int i = 0; i < 3; ++i)
		matrix[i] = new float[points.size()];

	float** matrixT = new float*[points.size()];
	for (unsigned int i = 0; i < points.size(); ++i)
		matrixT[i] = new float[3];
	//float **matrix = createMatrix(3, points.size());
	//float **matrixT = createMatrix(points.size(), 3);


	for (unsigned int i = 0; i < points.size(); i++)
	{
		auto centeredPoint = points[i] - centroid;
		matrix[0][i] = centeredPoint.x;
		matrix[1][i] = centeredPoint.y;
		matrix[2][i] = centeredPoint.z;

		matrixT[i][0] = centeredPoint.x;
		matrixT[i][1] = centeredPoint.y;
		matrixT[i][2] = centeredPoint.z;

		centeredPoints.push_back(points[i] - centroid);
	}


	auto covariance = glm::mat3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
	// Matrix multiplication
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			for (unsigned int k = 0; k < points.size(); ++k)
			{
				covariance[i][j] += matrix[i][k] * matrixT[k][j];
			}


	delete matrix;
	delete matrixT;

	// Normalization
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			covariance[i][j] /= points.size();

	return covariance;
}



