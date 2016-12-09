__kernel void covariance_matrix(__global float *vec, __global float *result, __local float *local_mem, int size, int align, int nextAlign)
{
	int global_id = (int)get_global_id(0);
	int align_id = global_id % align;
	int align_group_id = global_id / align;
	int group_id = (int)get_group_id(0);
	int local_id = (int)get_local_id(0);
	int local_w = (int)get_local_size(0);
	int align_groups = align / local_w;

	int a_offset, b_offset;
	float a, b;
	switch (align_group_id)
	{
	case 0:
		a_offset = 0; b_offset = 0;
		//a = neg_center.x; b = neg_center.x;
		break;
	case 1:
		a_offset = 0; b_offset = 1;
		//a = neg_center.x; b = neg_center.y;
		break;
	case 2:
		a_offset = 0; b_offset = 2;
		//a = neg_center.x; b = neg_center.z;
		break;
	case 3:
		a_offset = 1; b_offset = 1;
		//a = neg_center.y; b = neg_center.y;
		break;
	case 4:
		a_offset = 1; b_offset = 2;
		//a = neg_center.y; b = neg_center.z;
		break;
	case 5:
		a_offset = 2; b_offset = 2;
		//a = neg_center.z; b = neg_center.z;
		break;
	}

	a = vec[a_offset * align + align_id];
	b = vec[b_offset * align + align_id];
	
	local_mem[local_id] = a * b;

	barrier(CLK_LOCAL_MEM_FENCE);

	for (int working_size = (int)local_w >> 1; working_size > 0; working_size >>= 1)
	{
		if (local_id < working_size)
			local_mem[local_id] += local_mem[local_id + working_size];

		barrier(CLK_LOCAL_MEM_FENCE);
	}

	if (local_id == 0) {
		//result[(align_group_id * nextAlign) + (group_id % align_groups)] = local_mem[0];
		result[(align_group_id * nextAlign) + (group_id % align_groups)] = local_mem[0];
	}
}