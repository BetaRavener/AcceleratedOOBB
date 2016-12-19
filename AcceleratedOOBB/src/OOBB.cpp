#include "OOBB.h"
#include "glmext.h"


OOBB::OOBB()
{
}


OOBB::~OOBB()
{
}

std::ostream& operator<<(std::ostream &strm, const OOBB &obb)
{
	return strm << "Axes\n\t0: " << formatVec3(obb.axes[0])
		<< "\n\t1: " << formatVec3(obb.axes[1])
		<< "\n\t2: " << formatVec3(obb.axes[2])
		<< "\nDimensions:\n\tMin: " << obb.minimums[0] << ", " << obb.minimums[1] << ", " << obb.minimums[2]
		<< "\n\tMax: " << obb.maximums[0] << ", " << obb.maximums[1] << ", " << obb.maximums[2];
}
