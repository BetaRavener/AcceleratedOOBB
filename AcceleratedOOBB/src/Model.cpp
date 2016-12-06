#include "Model.h"
#include <fstream>

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

	return points;
}
