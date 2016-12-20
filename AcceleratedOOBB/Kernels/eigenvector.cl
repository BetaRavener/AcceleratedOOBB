#define M_PI 3.14159265359f

float determinant(float a, float b, float c, float d, float e, float f, float g, float h, float i)
{
	float x1 = a * e * i;
	float x2 = d * h * c;
	float x3 = g * b * f;

	float y1 = c * e * g;
	float y2 = d * b * i;
	float y3 = a * h * f;

	return x1 + x2 + x3 - y1 - y2 - y3;
}

__kernel void compute_eigens(__global float *covariance, __global float * eigens)
{
	const int global_id = (int)get_global_id(0);


	if (global_id == 0)
	{
		float a = covariance[0]; //[0][0]
		float b = covariance[1]; //[0][1]
		float c = covariance[2]; //[0][2]
		float d = b;			 //[1][0]
		float e = covariance[3]; //[1][1]
		float f = covariance[4]; //[1][2]
		float g = c;			 //[2][0]
		float h = f;			 //[2][1]
		float i = covariance[5]; //[2][2]

		float eigenValue1 = 0;
		float eigenValue2 = 0;
		float eigenValue3 = 0;

		// Compute eigen values
		float p1 = pow(b, 2) + pow(c, 2) + pow(f, 2);

		if (p1 == 0)
		{
			// Matrix is diagonal
			eigenValue1 = a;
			eigenValue2 = e;
			eigenValue3 = i;
		}
		else
		{
			float q = (a + e + i) / 3;
			float p2 = pow(a - q, 2) + pow(e - q, 2) + pow(i - q, 2) + 2 * p1;
			float p = sqrt(p2 / 6);
			
			//float B = (1 / p) * (covariance - q * glm::mat3x3(1, 0, 0, 0, 1, 0, 0, 0, 1));
			float r = determinant((1 / p) * (a - q), (1 / p) * b, (1 / p) * c, (1 / p) * d, 
				(1 / p) * (e - q), (1 / p) * f, (1 / p) * g, (1 / p) * h, (1 / p) * (i - q)) / 2;

			float phi = 0.0f;
			if (r <= -1)
			{
				phi = M_PI / 3;
			}
			else if (r >= 1)
			{
				phi = 0.0f;
			}
			else
			{
				phi = acos(r) / 3;
			}

			eigenValue1 = q + 2 * p * cos(phi);
			eigenValue3 = q + 2 * p * cos(phi + (2 * M_PI / 3));
			eigenValue2 = 3 * q - eigenValue1 - eigenValue3;
		}

		float x13 = 1.0f;
		float x11 = (b*f + c*(eigenValue1 - e))*x13 / ((eigenValue1 - a) * (eigenValue1 - e) - b*d);
		float x12 = (d*x11 + f*x13) / (eigenValue1 - e);

		float length = sqrt((x11 * x11) + (x12 * x12) + (x13 * x13));
		eigens[0] = x11 / length;
		eigens[1] = x12 / length;
		eigens[2] = x13 / length;


		x13 = 1.0f;
		x11 = (b*f + c*(eigenValue2 - e))*x13 / ((eigenValue2 - a) * (eigenValue2 - e) - b*d);
		x12 = (d*x11 + f*x13) / (eigenValue2 - e);

		length = sqrt((x11 * x11) + (x12 * x12) + (x13 * x13));
		eigens[3] = x11 / length;
		eigens[4] = x12 / length;
		eigens[5] = x13 / length;

		x13 = 1.0f;
		x11 = (b*f + c*(eigenValue3 - e))*x13 / ((eigenValue3 - a) * (eigenValue3 - e) - b*d);
		x12 = (d*x11 + f*x13) / (eigenValue3 - e);

		length = sqrt((x11 * x11) + (x12 * x12) + (x13 * x13));
		eigens[6] = x11 / length;
		eigens[7] = x12 / length;
		eigens[8] = x13 / length;
	}


}
