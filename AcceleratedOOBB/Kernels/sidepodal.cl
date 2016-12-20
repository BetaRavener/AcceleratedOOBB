// The kernel should be run with local group size of at least 16x16 for correct
// coalescing access pattern. The group size must be power of 2 and at least 4.
__kernel void sidepodal(__global float4 *normals, __global uchar *result, __local float4 *local_mem, int size)
{
	int g_x = (int)get_global_id(0);
	int g_y = (int)get_global_id(1);
	int l_x = (int)get_local_id(0);
	int l_y = (int)get_local_id(1);

	// Discard threads larger than input size
	if (g_x >= size || g_y >= size)
		return;

	// If the first (top left) member of group gets discarded,
	// whole group gets discarded early on as there's no work to do
	if ((g_x - l_x) < (g_y - l_y)) {
		return;
	}

	int l_s = (int)get_local_size(0); // always square

	// Otherwise load data even if the thread won't run calculations.
	// Local memory is linear to save up space.
	// There would be 2 float4 to load per thread (2 normals),
	// so split that betweeen other threads.
	if (l_y < 2) { // Load row
		local_mem[l_x + l_y * l_s] = normals[g_x + l_y * l_s];
	}
	else if (l_y < 4) { // Load column
		int y = (g_y - l_y) + l_x + (l_y - 2) * l_s;
		local_mem[l_x + l_y * l_s] = normals[y];
	}

	barrier(CLK_LOCAL_MEM_FENCE);

	// Discard all left over, non-working threads 
	if (g_x < g_y)
		return;

	// Load needed components
	float4 f1a = local_mem[l_x * 2];
	float4 f1b = local_mem[(l_x * 2) + 1];
	float4 f2a = local_mem[(l_s * 2) + (l_y * 2)];
	float4 f2b = local_mem[(l_s * 2) + (l_y * 2) + 1];

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
