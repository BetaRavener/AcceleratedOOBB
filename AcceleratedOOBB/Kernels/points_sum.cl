__kernel void points_sum(__global float *points, __local float *local_mem, __global float *result, int length, int align)
{
	const int global_id = (int)get_global_id(0);
	const int group_id = (int)get_group_id(0);
	const int local_id = (int)get_local_id(0);
	const int local_w = (int)get_local_size(0);

	const int align_groups = align / local_w;
	const int groupsCountPerDimension = align_groups / 3;
	const int globalSizePerDimension = align / 3;
	const int dimension = group_id / groupsCountPerDimension;

	float sum = 0;
	int current_id = global_id - dimension * globalSizePerDimension;
	// Loop sequentially over chunks of input vector
	while (current_id < length) {
		sum += points[dimension * length + current_id];
		current_id += globalSizePerDimension;
	}

	// Perform parallel reduction
	local_mem[local_id] = sum;
	barrier(CLK_LOCAL_MEM_FENCE);

	for (int working_size = local_w >> 1; working_size > 0; working_size >>= 1)
	{
		if (local_id < working_size)
		{
			local_mem[local_id] += local_mem[local_id + working_size];
		}

		barrier(CLK_LOCAL_MEM_FENCE);
	}

	if (local_id == 0) {
		result[group_id] = local_mem[local_id];
	}
}