float findMin(float a, float b)
{
	return a < b ? a : b;
}

float findMax(float a, float b)
{
	return a > b ? a : b;
}

__kernel void projection_matrix(__global float *vec, __global float *result, __global float * eigens, __local float *local_eigens, __local float *local_mem,
	int pointsCount, int threadCount, int alignedPointsCount)
{
	const int global_id = (int)get_global_id(0);
	const int local_id = (int)get_local_id(0);
	const int group_id = (int)get_group_id(0);

	if (local_id < 9)
	{
		local_eigens[local_id] = eigens[local_id];
	}

	barrier(CLK_LOCAL_MEM_FENCE);

	float min_x = 0;
	float min_y = 0;
	float min_z = 0;

	float max_x = 0;
	float max_y = 0;
	float max_z = 0;

	int current_id = global_id;
	// Loop sequentially over chunks of input points
	while (current_id < pointsCount) {
		float x = vec[0 * alignedPointsCount + current_id];
		float y = vec[1 * alignedPointsCount + current_id];
		float z = vec[2 * alignedPointsCount + current_id];

		float projected_x = x * local_eigens[0] + y * local_eigens[1] + z * local_eigens[2];
		float projected_y = x * local_eigens[3] + y * local_eigens[4] + z * local_eigens[5];
		float projected_z = x * local_eigens[6] + y * local_eigens[7] + z * local_eigens[8];

		min_x = findMin(min_x, projected_x);
		min_y = findMin(min_y, projected_y);
		min_z = findMin(min_z, projected_z);

		max_x = findMax(max_x, projected_x);
		max_y = findMax(max_y, projected_y);
		max_z = findMax(max_z, projected_z);

		current_id += threadCount;
	}


	local_mem[0 + 6 * local_id] = min_x;
	local_mem[1 + 6 * local_id] = min_y;
	local_mem[2 + 6 * local_id] = min_z;

	local_mem[3 + 6 * local_id] = max_x;
	local_mem[4 + 6 * local_id] = max_y;
	local_mem[5 + 6 * local_id] = max_z;

	barrier(CLK_LOCAL_MEM_FENCE);

	for (int working_size = (int)get_local_size(0) >> 1; working_size > 0; working_size >>= 1)
	{
		if (local_id < working_size)
		{
			local_mem[0 + 6 * local_id] = findMin(local_mem[0 + 6 * local_id], local_mem[0 + 6 * local_id + 6 * working_size]);
			local_mem[1 + 6 * local_id] = findMin(local_mem[1 + 6 * local_id], local_mem[1 + 6 * local_id + 6 * working_size]);
			local_mem[2 + 6 * local_id] = findMin(local_mem[2 + 6 * local_id], local_mem[2 + 6 * local_id + 6 * working_size]);

			local_mem[3 + 6 * local_id] = findMax(local_mem[3 + 6 * local_id], local_mem[3 + 6 * local_id + 6 * working_size]);
			local_mem[4 + 6 * local_id] = findMax(local_mem[4 + 6 * local_id], local_mem[4 + 6 * local_id + 6 * working_size]);
			local_mem[5 + 6 * local_id] = findMax(local_mem[5 + 6 * local_id], local_mem[5 + 6 * local_id + 6 * working_size]);
		}

		barrier(CLK_LOCAL_MEM_FENCE);
	}

	if (local_id < 3) {
		result[6 * group_id + local_id] = local_mem[local_id];

		result[ 6 * group_id + local_id + 3] = local_mem[3 + local_id];
	}
}

__kernel void reduction_minmax(__global float *minMax, __global float *result, __local float *local_mem, int size, int align)
{
	const int global_id = (int)get_global_id(0);
	const int local_id = (int)get_local_id(0);
	const int group_id = (int)get_group_id(0);
	const int pointsPerBlock = (int)get_local_size(0);
		
	if (global_id % align < size)
	{
		local_mem[0 + 6 * local_id] = minMax[group_id * pointsPerBlock + 6 * local_id + 0];
		local_mem[1 + 6 * local_id] = minMax[group_id * pointsPerBlock + 6 * local_id + 1];
		local_mem[2 + 6 * local_id] = minMax[group_id * pointsPerBlock + 6 * local_id + 2];

		local_mem[3 + 6 * local_id] = minMax[group_id * pointsPerBlock + 6 * local_id + 3];
		local_mem[4 + 6 * local_id] = minMax[group_id * pointsPerBlock + 6 * local_id + 4];
		local_mem[5 + 6 * local_id] = minMax[group_id * pointsPerBlock + 6 * local_id + 5];
	}
	else
	{
		local_mem[0 + 6 * local_id] = 0;
		local_mem[1 + 6 * local_id] = 0;
		local_mem[2 + 6 * local_id] = 0;

		local_mem[3 + 6 * local_id] = 0;
		local_mem[4 + 6 * local_id] = 0;
		local_mem[5 + 6 * local_id] = 0;
	}

	barrier(CLK_LOCAL_MEM_FENCE);

	for (int working_size = pointsPerBlock >> 1; working_size > 0; working_size >>= 1)
	{
		if (local_id < working_size)
		{
			local_mem[0 + 6 * local_id] = findMin(local_mem[0 + 6 * local_id], local_mem[0 + 6 * local_id + 6 * working_size]);
			local_mem[1 + 6 * local_id] = findMin(local_mem[1 + 6 * local_id], local_mem[1 + 6 * local_id + 6 * working_size]);
			local_mem[2 + 6 * local_id] = findMin(local_mem[2 + 6 * local_id], local_mem[2 + 6 * local_id + 6 * working_size]);

			local_mem[3 + 6 * local_id] = findMax(local_mem[3 + 6 * local_id], local_mem[3 + 6 * local_id + 6 * working_size]);
			local_mem[4 + 6 * local_id] = findMax(local_mem[4 + 6 * local_id], local_mem[4 + 6 * local_id + 6 * working_size]);
			local_mem[5 + 6 * local_id] = findMax(local_mem[5 + 6 * local_id], local_mem[5 + 6 * local_id + 6 * working_size]);
		}

		barrier(CLK_LOCAL_MEM_FENCE);
	}

	if (local_id < 3) {
		result[6 * group_id + local_id] = local_mem[local_id];

		result[6 * group_id + local_id + 3] = local_mem[3 + local_id];
	}
}
