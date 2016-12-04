#include "Generator.h"
#include <glm/detail/func_geometric.inl>
#include <random>

Generator::Generator(glm::vec3 min, glm::vec3 max, glm::vec3 axis)
{
	_min = min;
	_max = max;
	_axis = glm::normalize(axis);
	_up = glm::vec3(1, 0, 0);
	_right = glm::cross(_axis, _up);
	_up = glm::cross(_axis, _right);
}

std::vector<glm::vec3> Generator::CreatePointCloud(int count) const
{
	std::vector<glm::vec3> pointCloud;
	pointCloud.reserve(count);

	// Initialize random generators;
	std::random_device rd; // obtain a random number from hardware
	std::mt19937 eng(rd()); // seed the generator
	std::uniform_real_distribution<> distrX(_min.x, _max.x); // define the range
	std::uniform_real_distribution<> distrY(_min.y, _max.y); // define the range
	std::uniform_real_distribution<> distrZ(_min.z, _max.z); // define the range

	for (auto i = 0; i < count; i++)
	{
		// Generate point inside the range specified by bounding box
		auto x = static_cast<float>(distrX(eng));
		auto y = static_cast<float>(distrY(eng));
		auto z = static_cast<float>(distrZ(eng));
		
		// Transform it to new coordinate system
		auto point = (x * _axis) + (y * _up) + (z * _right);

		// Add point to cloud
		pointCloud.push_back(point);
	}	

	return pointCloud;
}
