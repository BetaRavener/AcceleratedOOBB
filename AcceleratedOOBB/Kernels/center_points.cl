__kernel void center_points(__global float *points, float centroid_x, float centroid_y, float centroid_z, int length, int align, int pointsCount)
{
	const int global_id = (int)get_global_id(0);
	const int group_id = (int)get_group_id(0);
	const int local_id = (int)get_local_id(0);
	const int local_w = (int)get_local_size(0);

	const int align_groups = align / local_w;
	const int groupsCountPerDimension = align_groups / 3;
	const int globalSizePerDimension = align / 3;
	const int dimension = group_id / groupsCountPerDimension;

	float centroid_;
	switch (dimension)
	{
	case 0:
		centroid_ = centroid_x;
		break;
	case 1:
		centroid_ = centroid_y;
		break;
	case 2:
		centroid_ = centroid_z;
		break;
	}

	int current_id = global_id - dimension * globalSizePerDimension;
	while (current_id < pointsCount) {
		points[dimension * length + current_id] = points[dimension * length + current_id] - centroid_;
		current_id += globalSizePerDimension;
	}
}