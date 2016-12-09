#include "Cpu.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include "eig3.h"
#include <fstream>


std::vector<glm::vec3> Cpu::CreateEigens(std::vector<glm::vec3> & points)
{
	auto centroid = ComputeCentroid(points);

	auto covarianceMatrix = ComputeCovarianceMatrix(points, centroid);

	auto eigenValues = ComputeEigenValues(covarianceMatrix);
	auto eigenVecs = ComputeEigenVectors(covarianceMatrix, eigenValues);

	// Original eigenvector / eigenvalue computation
	double matrix[3][3];

	matrix[0][0] = covarianceMatrix[0][0];
	matrix[0][1] = covarianceMatrix[0][1];
	matrix[0][2] = covarianceMatrix[0][2];
	matrix[1][0] = covarianceMatrix[1][0];
	matrix[1][1] = covarianceMatrix[1][1];
	matrix[1][2] = covarianceMatrix[1][2];
	matrix[2][0] = covarianceMatrix[2][0];
	matrix[2][1] = covarianceMatrix[2][1];
	matrix[2][2] = covarianceMatrix[2][2];

	double eigenVectors[3][3];
	double values[3];

	eigen_decomposition(matrix, eigenVectors, values);

	//auto eigenVector1 = glm::vec3(eigenVectors[0][0], eigenVectors[1][0], eigenVectors[2][0]);
	//auto eigenVector2 = glm::vec3(eigenVectors[0][1], eigenVectors[1][1], eigenVectors[2][1]);
	//auto eigenVector3 = glm::cross(eigenVector1, eigenVector2);

	return eigenVecs;
}

OOBB Cpu::CreateOOBB(std::vector<glm::vec3> & points)
{ 
	auto centroid = ComputeCentroid(points);

	auto covarianceMatrix = ComputeCovarianceMatrix(points, centroid);

	auto eigenValues = ComputeEigenValues(covarianceMatrix);
	auto eigenVecs = ComputeEigenVectors(covarianceMatrix, eigenValues);

	// Original eigenvector / eigenvalue computation
	double matrix[3][3];

	matrix[0][0] = covarianceMatrix[0][0];
	matrix[0][1] = covarianceMatrix[0][1];
	matrix[0][2] = covarianceMatrix[0][2];
	matrix[1][0] = covarianceMatrix[1][0];
	matrix[1][1] = covarianceMatrix[1][1];
	matrix[1][2] = covarianceMatrix[1][2];
	matrix[2][0] = covarianceMatrix[2][0];
	matrix[2][1] = covarianceMatrix[2][1];
	matrix[2][2] = covarianceMatrix[2][2];

	double eigenVectors[3][3];
	double values[3];

	eigen_decomposition(matrix, eigenVectors, values);
	
	//auto eigenVector1 = glm::vec3(eigenVectors[0][0], eigenVectors[1][0], eigenVectors[2][0]);
	//auto eigenVector2 = glm::vec3(eigenVectors[0][1], eigenVectors[1][1], eigenVectors[2][1]);
	//auto eigenVector3 = glm::cross(eigenVector1, eigenVector2);

	auto eigenVector1 = eigenVecs[0];
	auto eigenVector2 = eigenVecs[1];
	auto eigenVector3 = eigenVecs[2];

	auto min = glm::vec3();
	auto max = glm::vec3();
	for (unsigned int i = 0; i < points.size(); i++)
	{
		auto centeredPoint = (points[i] - centroid);

		auto projectedPoint = glm::vec3();
		projectedPoint.x = glm::dot(centeredPoint, eigenVector1);
		projectedPoint.y = glm::dot(centeredPoint, eigenVector2);
		projectedPoint.z = glm::dot(centeredPoint, eigenVector3);

		if (projectedPoint.x < min.x)
			min.x = projectedPoint.x;
		if (projectedPoint.y < min.y)
			min.y = projectedPoint.y;
		if (projectedPoint.z < min.z)
			min.z = projectedPoint.z;

		if (projectedPoint.x > max.x)
			max.x = projectedPoint.x;
		if (projectedPoint.y > max.y)
			max.y = projectedPoint.y;
		if (projectedPoint.z > max.z)
			max.z = projectedPoint.z;
	}

	// Finding Min and Max

	auto result = OOBB();
	result.center = centroid;

	result.maximums[0] = max.x;
	result.maximums[1] = max.y;
	result.maximums[2] = max.z;

	result.minimums[0] = min.x;
	result.minimums[1] = min.y;
	result.minimums[2] = min.z;

	result.axes[0] = eigenVector1;
	result.axes[1] = eigenVector2;
	result.axes[2] = eigenVector3;

	auto fout = std::ofstream("eigv.txt", std::ofstream::app);
	fout << eigenValues[0] << " " << eigenValues[1] << " " << eigenValues[2] << std::endl;
	fout << eigenVector1[0] << " " << eigenVector1[1] << " " << eigenVector1[2] << std::endl;
	fout << eigenVector2[0] << " " << eigenVector2[1] << " " << eigenVector2[2] << std::endl;
	fout << eigenVector3[0] << " " << eigenVector3[1] << " " << eigenVector3[2] << std::endl;
	fout << "-----" << std::endl;
	fout.close();

	return result;
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

glm::dmat3x3 Cpu::ComputeCovarianceMatrix(std::vector<glm::vec3> & points, glm::vec3 centroid)
{
	std::vector<glm::vec3> centeredPoints(points.size());
	double** matrix = new double*[3];
	for (unsigned int i = 0; i < 3; ++i)
		matrix[i] = new double[points.size()];

	double** matrixT = new double*[points.size()];
	for (unsigned int i = 0; i < points.size(); ++i)
		matrixT[i] = new double[3];

	for (unsigned int i = 0; i < points.size(); i++)
	{
		auto centeredPoint = points[i] - centroid;
		matrix[0][i] = centeredPoint.x;
		matrix[1][i] = centeredPoint.y;
		matrix[2][i] = centeredPoint.z;

		matrixT[i][0] = centeredPoint.x;
		matrixT[i][1] = centeredPoint.y;
		matrixT[i][2] = centeredPoint.z;

		centeredPoints[i] = (points[i] - centroid);
	}


	auto covariance = glm::dmat3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
	// Matrix multiplication
	for (int i = 0; i < 3; ++i)	{
		for (int j = 0; j < 3; ++j) {
			for (unsigned int k = 0; k < points.size(); ++k){
				covariance[i][j] += matrix[i][k] * matrix[j][k];
			}
		}
	}

	delete[] matrix;
	delete[] matrixT;

	// Normalization
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			covariance[i][j] /= points.size() - 1;

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

std::vector<glm::vec3> Cpu::ComputeEigenVectors(glm::mat3x3 covariance, glm::vec3 eigv)
{
	auto res = std::vector<glm::vec3>();
	auto a = covariance[0][0];
	auto b = covariance[0][1];
	auto c = covariance[0][2];
	auto d = covariance[1][0];
	auto e = covariance[1][1];
	auto f = covariance[1][2];
	auto g = covariance[2][0];
	auto h = covariance[2][1];
	auto i = covariance[2][2];

	auto val = eigv.x;
	auto x13 = 1.0f;
	auto x11 = (b*f + c*(val - e))*x13 / ((val - a) * (val - e) - b*d);
	auto x12 = (d*x11 + f*x13) / (val - e);
	auto vec = glm::vec3(x11, x12, x13);
	vec = glm::normalize(vec);
	res.push_back(vec);


	val = eigv.y;
	x13 = 1.0f;
	x11 = (b*f + c*(val - e))*x13 / ((val - a) * (val - e) - b*d);
	x12 = (d*x11 + f*x13) / (val - e);
	vec = glm::vec3(x11, x12, x13);
	vec = glm::normalize(vec);
	res.push_back(vec);

	val = eigv.z;
	x13 = 1.0f;
	x11 = (b*f + c*(val - e))*x13 / ((val - a) * (val - e) - b*d);
	x12 = (d*x11 + f*x13) / (val - e);
	vec = glm::vec3(x11, x12, x13);
	vec = glm::normalize(vec);
	res.push_back(vec);

	//res.push_back(glm::cross(res[0], res[1]));

	return res;
}


