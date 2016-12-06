_INLINE_ void atomicAdd_g_f(volatile __global float *addr, float val)
{
	union {
		unsigned int u32;
		float        f32;
	} next, expected, current;

	current.f32 = *addr;
	do {
		expected.f32 = current.f32;
		next.f32 = expected.f32 + val;

		// If the current value is not same as expected, write occured and
		// adding float needs to be repeated
		current.u32 = atomic_cmpxchg((volatile __global unsigned int *)addr,
			expected.u32, next.u32);
	} while (current.u32 != expected.u32);
}

__kernel void atomic_reduce(__global float *vec, __global float *result, __local float *local_mem)
{
	int global_id = (int)get_global_id(0);
	int local_id = (int)get_local_id(0);
	int local_w = (int)get_local_size(0);

	if (global_id >= array_size)
		local_mem[local_id] = 0;
	else
		local_mem[local_id] = vec[global_id];

	barrier(CLK_LOCAL_MEM_FENCE);

	for (int working_size = local_w >> 2; working_size > 0; working_size >>= 2)
	{
		if (local_id < working_size)
			local_mem[local_id] += local_mem[local_id + working_size];

		barrier(CLK_LOCAL_MEM_FENCE);
	}

	if (local_id == 0)
		atomicAdd_g_f(result, local_mem[0]);
}