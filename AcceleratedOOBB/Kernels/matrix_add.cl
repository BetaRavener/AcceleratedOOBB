__kernel void matrix_add(__global int *a, __global int *b, __global int *c, int width, int height)
{
	int global_x = (int)get_global_id(0);
	int global_y = (int)get_global_id(1);
	//===========================================================================================  
	/* ======================================================
	* TODO 6. Cast
	* doplnit telo kernelu
	* =======================================================
	*/
	if (global_x >= width)
		return;
	if (global_y >= height)
		return;
	int idx = global_y * width + global_x;
	c[idx] = a[idx] + b[idx];
}