#include "Cpu.h"

#define _USE_MATH_DEFINES
#include <math.h>


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

	auto eigenValues = ComputeEigenValues(covarianceMatrix);

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

float traceMatrix(glm::mat3x3 matrix)
{
	return matrix[0][0] + matrix[1][1] + matrix[2][2];
}

float determinant(glm::mat3x3 matrix)
{
	auto x1 = matrix[0][0] * matrix[1][1] * matrix[2][2];
	auto x2 = matrix[1][0] * matrix[2][1] * matrix[0][2];
	auto x3 = matrix[2][0] * matrix[0][1] * matrix[1][2];

	auto y1 = matrix[0][2] * matrix[1][1] * matrix[2][0];
	auto y2 = matrix[1][0] * matrix[0][1] * matrix[2][2];
	auto y3 = matrix[0][0] * matrix[2][1] * matrix[1][2];

	return x1 + x2 + x3 - y1 - y2 - y3;
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

glm::vec3 Cpu::ComputeEigenValues(glm::mat3x3 covariance)
{
	//% Given a real symmetric 3x3 matrix A, compute the eigenvalues

	//	p1 = A(1, 2) ^ 2 + A(1, 3) ^ 2 + A(2, 3) ^ 2
	//	if (p1 == 0)
	//		% A is diagonal.
	//		eig1 = A(1, 1)
	//		eig2 = A(2, 2)
	//		eig3 = A(3, 3)
	//	else
	//		q = trace(A) / 3
	//		p2 = (A(1, 1) - q) ^ 2 + (A(2, 2) - q) ^ 2 + (A(3, 3) - q) ^ 2 + 2 * p1
	//		p = sqrt(p2 / 6)
	//		B = (1 / p) * (A - q * I) % I is the identity matrix
	//		r = det(B) / 2

	//		% In exact arithmetic for a symmetric matrix - 1 <= r <= 1
	//		% but computation error can leave it slightly outside this range.
	//		if (r <= -1)
	//			phi = pi / 3
	//			elseif(r >= 1)
	//			phi = 0
	//		else
	//			phi = acos(r) / 3
	//			end

	//			% the eigenvalues satisfy eig3 <= eig2 <= eig1
	//			eig1 = q + 2 * p * cos(phi)
	//			eig3 = q + 2 * p * cos(phi + (2 * pi / 3))
	//			eig2 = 3 * q - eig1 - eig3     % since trace(A) = eig1 + eig2 + eig3
	//			end
	auto p1 = pow(covariance[0][1], 2) + pow(covariance[0][2], 2) + pow(covariance[1][2], 2);

	if (p1 == 0)
	{
		// Matrix is diagonal
		return glm::vec3(covariance[0][0], covariance[1][1], covariance[2][2]);
	}
	else
	{
		auto q = traceMatrix(covariance) / 3;
		auto p2 = pow(covariance[0][0] - q, 2) + pow(covariance[1][1] - q, 2) + pow(covariance[2][2] - q, 2) + 2 * p1;
		auto p = sqrt(p2 / 6);
		auto B = (1 / p) * (covariance - q * glm::mat3x3(1, 0, 0, 0, 1, 0, 0, 0, 1));
		auto r = determinant(B) / 2;

		auto phi = 0.0;
		if (r <= -1)
		{
			phi = M_PI / 3;
		}
		else if (r >= 1)
		{
			phi = 0.0;
		}
		else
		{
			phi = acos(r) / 3;
		}

		auto eig1 = q + 2 * p * cos(phi);
		auto eig3 = q + 2 * p * cos(phi + (2 * M_PI / 3));
		auto eig2 = 3 * q - eig1 - eig3;
		return glm::vec3(eig1, eig2, eig3);
	}
}




