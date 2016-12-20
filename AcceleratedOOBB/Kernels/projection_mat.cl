float findMin(float a, float b)
{
	return a < b ? a : b;
}

float findMax(float a, float b)
{
	return a > b ? a : b;
}

__kernel void projection_matrix(__global float *vec, __global float *result, __global float * eigens, __local float *local_eigens, __local float *local_mem, int size, int align)
{
	const int global_id = (int)get_global_id(0);
	const int align_id = global_id % align;
	const int local_id = (int)get_local_id(0);
	const int group_id = (int)get_group_id(0);
	const int valuesCount = align / (int)get_local_size(0);

	if (local_id < 9)
	{
		local_eigens[local_id] = eigens[local_id];
	}

	float x = vec[0 * align + global_id];
	float y = vec[1 * align + global_id];
	float z = vec[2 * align + global_id];

	barrier(CLK_LOCAL_MEM_FENCE);

	float projected_x = x * local_eigens[0] + y * local_eigens[1] + z * local_eigens[2];
	float projected_y = x * local_eigens[3] + y * local_eigens[4] + z * local_eigens[5];
	float projected_z = x * local_eigens[6] + y * local_eigens[7] + z * local_eigens[8];


	local_mem[0 + 6 * local_id] = projected_x;
	local_mem[1 + 6 * local_id] = projected_y;
	local_mem[2 + 6 * local_id] = projected_z;

	local_mem[3 + 6 * local_id] = projected_x;
	local_mem[4 + 6 * local_id] = projected_y;
	local_mem[5 + 6 * local_id] = projected_z;

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
