__kernel void projection_matrix(__global float *vec, __global float *result, __global float * eigens, __local float *local_mem, int size, int align)
{
	int global_id = (int)get_global_id(0);
	int align_id = global_id % align;
	int local_id = (int)get_local_id(0);
	int group_id = (int)get_group_id(0);
	int valuesCount = align / (int)get_local_size(0);

	if (local_id < 9)
	{
		local_mem[local_id] = eigens[local_id];
	}

	barrier(CLK_LOCAL_MEM_FENCE);

	int a_offset, b_offset;
	float x = vec[0 * align + global_id];
	float y = vec[1 * align + global_id];
	float z = vec[2 * align + global_id];

	float projected_x = x * local_mem[0] + y * local_mem[1] + z * local_mem[2];
	float projected_y = x * local_mem[3] + y * local_mem[4] + z * local_mem[5];
	float projected_z = x * local_mem[6] + y * local_mem[7] + z * local_mem[8];


	local_mem[9 + 6 * local_id] = projected_x;
	local_mem[10 + 6 * local_id] = projected_y;
	local_mem[11 + 6 * local_id] = projected_z;


	barrier(CLK_LOCAL_MEM_FENCE);

	for (int working_size = (int)get_local_size(0) >> 1; working_size > 0; working_size >>= 1)
	{
		if (local_x < working_size)
		{
			local_mem[9 + 6 * local_x] = min(local_mem[9 + 6 * local_x], local_mem[9 + 6 * local_x + working_size]);
			local_mem[10 + 6 * local_x] = min(local_mem[10 + 6 * local_x], local_mem[10 + 6 * local_x + working_size]);
			local_mem[11 + 6 * local_x] = min(local_mem[11 + 6 * local_x], local_mem[11 + 6 * local_x + working_size]);

			local_mem[12 + 6 * local_x] = max(local_mem[12 + 6 * local_x], local_mem[12 + 6 * local_x + working_size]);
			local_mem[13 + 6 * local_x] = max(local_mem[13 + 6 * local_x], local_mem[13 + 6 * local_x + working_size]);
			local_mem[14 + 6 * local_x] = max(local_mem[14 + 6 * local_x], local_mem[14 + 6 * local_x + working_size]);
		}

		barrier(CLK_LOCAL_MEM_FENCE);
	}

	if (local_id == 0) {
		result[3 * group_id] = local_mem[0];
		result[3 * group_id + 1] = local_mem[1];
		result[3 * group_id + 2] = local_mem[2];

		result[3 * valuesCount + 3 * group_id] = local_mem[3];
		result[3 * valuesCount + 3 * group_id + 1] = local_mem[4];
		result[3 * valuesCount + 3 * group_id + 2] = local_mem[5];
	}
}