#include "Model.h"
#include <fstream>
#include "Cpu.h"

std::vector<glm::vec3> Model::load(std::string fileName, float scale)
{
	static std::string prefix = "../Models/";
	auto fin = std::ifstream(prefix + fileName);

	std::vector<glm::vec3> points;
	float x, y, z;
	while(fin.good())
	{
		fin >> x >> y >> z;
		points.push_back(glm::vec3(x * scale, y * scale, z * scale));
	}

	// Center the model into origin
	glm::vec3 centroid = Cpu::ComputeCentroid(points);
	for (auto i = 0; i < points.size(); i++)
		points[i] -= centroid;

	return points;
}
