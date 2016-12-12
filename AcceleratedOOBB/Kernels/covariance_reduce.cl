//_INLINE_ void atomicAdd_g_f(volatile __global float *addr, float val)
//{
//	union {
//		unsigned int u32;
//		float        f32;
//	} next, expected, current;
//
//	current.f32 = *addr;
//	do {
//		expected.f32 = current.f32;
//		next.f32 = expected.f32 + val;
//
//		// If the current value is not same as expected, write occured and
//		// adding float needs to be repeated
//		current.u32 = atomic_cmpxchg((volatile __global unsigned int *)addr,
//			expected.u32, next.u32);
//	} while (current.u32 != expected.u32);
//}

__kernel void cov_reduce(__global float *vec, __global float *result, __local float *local_mem, int size, int align, int nextAlign, float multiplier)
{
	int global_id = (int)get_global_id(0);
	int align_id = global_id / align;
	int group_id = (int)get_group_id(0);
	int local_id = (int)get_local_id(0);
	int local_w = (int)get_local_size(0);
	int align_groups = align / local_w;

	if (global_id % align >= size)
		local_mem[local_id] = 0;
	else
		local_mem[local_id] = vec[global_id];

	barrier(CLK_LOCAL_MEM_FENCE);

	for (int working_size = local_w >> 1; working_size > 0; working_size >>= 1)
	{
		if (local_id < working_size)
			local_mem[local_id] += local_mem[local_id + working_size];

		barrier(CLK_LOCAL_MEM_FENCE);
	}

	if (local_id == 0) {
		result[align_id * nextAlign + group_id % align_groups] = local_mem[0] * multiplier;
	}
}