#include "glmext.h"

std::string formatVec3(const glm::vec3& vec)
{
	std::ostringstream oss;
	oss << vec.x << ", " << vec.y << ", " << vec.z;
	return oss.str();
}
