// The kernel should be run with local group size of at least 16x16 for correct
// coalescing access pattern. The group size must be power of 2 and at least 4.
__kernel void sidepodal(__global float4 *normals, __global uchar *result, __local float4 *local_mem, int size)
//__kernel void sidepodal(__global float4 *normals, __global float4 *result, __local float4 *local_mem, int size)
{
	int g_x = (int)get_global_id(0);
	int g_y = (int)get_global_id(1);
	int l_x = (int)get_local_id(0);
	int l_y = (int)get_local_id(1);
	int grp_x = (int)get_group_id(0);
	int grp_y = (int)get_group_id(1);

	// If the group is under diagonal it gets discarded
	// early on as there's no work to do
	if (grp_x < grp_y) {
		return;
	}

	int l_s = (int)get_local_size(0); // always square

	// Otherwise load data even if the thread won't run calculations.
	// Local memory is linear to save up space.
	// There would be 2 float4 to load per thread (2 normals),
	// so split that betweeen other threads.
	if (l_y < 2) { // Load row
		local_mem[l_x + l_y * l_s] = normals[(grp_x * 2 + l_y) * l_s + l_x];
	}
	else if (l_y < 4) { // Load column
		int y = (grp_y * 2 + l_y - 2) * l_s + l_x;
		local_mem[l_x + l_y * l_s] = normals[y];
	}

	barrier(CLK_LOCAL_MEM_FENCE);

	// Discard threads larger than input size
	// and all left over threads under diagonal.
	if (g_x >= size || g_y >= size || g_x < g_y)
		return;

	// Load needed components
	// F1 is indexed by Y component in original algorithm
	float4 f1a = local_mem[(l_s * 2) + (l_y * 2)];
	float4 f1b = local_mem[(l_s * 2) + (l_y * 2) + 1];
	// F2 is indexed by X component in original algorithm
	float4 f2a = local_mem[l_x * 2];
	float4 f2b = local_mem[(l_x * 2) + 1];

	/*result[0 + (g_x + g_y * size)*4] = f1a;
	result[1 + (g_x + g_y * size)*4] = f1b;
	result[2 + (g_x + g_y * size)*4] = f2a;
	result[3 + (g_x + g_y * size)*4] = f2b;*/

	float4 f1a_f1b = f1a - f1b;
	float4 f2a_f2b = f2a - f2b;
	float a = dot(f1b, f2b);
	float b = dot(f1a_f1b, f2b);
	float c = dot(f2a_f2b, f1b);
	float d = dot(f1a_f1b, f2a_f2b);

	float ab = a + b;
	float ac = a + c;
	float abcd = ab + c + d;

	result[g_x + g_y * size] = (min(min(a, ab), min(ac, abcd)) <= 0.f && max(max(a, ab), max(ac, abcd)) >= 0.f) ? 1 : 0;
}
