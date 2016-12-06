__kernel void covariance_matrix(__global float *vec, __global float *result, __local float *local_mem, float3 neg_center, int size, int align)
{
	//TODO: Pad each coordinate with 0 to align at max work_group size (512)
	//TODO: Center should be already negative
	int global_id = (int)get_global_id(0);
	int align_id = global_id % align;
	int local_id = (int)get_local_id(0);

	int a_offset, b_offset;
	float a, b;
	switch (global_id / align)
	{
	case 0:
		a_offset = 0; b_offset = 0;
		a = neg_center.x; b = neg_center.x;
		break;
	case 1:
		a_offset = 0; b_offset = 1;
		a = neg_center.x; b = neg_center.y;
		break;
	case 2:
		a_offset = 0; b_offset = 2;
		a = neg_center.x; b = neg_center.z;
		break;
	case 3:
		a_offset = 1; b_offset = 1;
		a = neg_center.y; b = neg_center.y;
		break;
	case 4:
		a_offset = 1; b_offset = 2;
		a = neg_center.y; b = neg_center.z;
		break;
	case 5:
		a_offset = 2; b_offset = 2;
		a = neg_center.z; b = neg_center.z;
		break;
	}

	a += vec[a_offset * align + align_id];
	b += vec[b_offset * align + align_id];
	
	local_mem[local_id] = a * b;

	barrier(CLK_LOCAL_MEM_FENCE);

	for (int working_size = (int)get_local_size(0) >> 1; working_size > 0; working_size >>= 1)
	{
		if (local_x < working_size)
			tmp_a[local_x] += tmp_a[local_x + working_size];

		barrier(CLK_LOCAL_MEM_FENCE);
	}

	if (local_id == 0) {
		result[get_group_id(0)] = tmp_a[0];
	}
}