/* Copyright Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file float3x3.cpp
	@author Jukka Jylänki
	@brief */
#include "float3x3.h"
#include "glmext.h"

float3x3::float3x3(float _00, float _01, float _02,
                   float _10, float _11, float _12,
                   float _20, float _21, float _22)
{
	Set(_00, _01, _02,
	    _10, _11, _12,
	    _20, _21, _22);
}

void float3x3::Set(float _00, float _01, float _02,
	float _10, float _11, float _12,
	float _20, float _21, float _22)
{
	v[0][0] = _00; v[0][1] = _01; v[0][2] = _02;
	v[1][0] = _10; v[1][1] = _11; v[1][2] = _12;
	v[2][0] = _20; v[2][1] = _21; v[2][2] = _22;
}

void float3x3::SetCol(int column, float x, float y, float z)
{
	v[0][column] = x;
	v[1][column] = y;
	v[2][column] = z;
}

void float3x3::SetCol(int column, const vec &columnVector)
{
	SetCol(column, columnVector.x, columnVector.y, columnVector.z);
}

bool float3x3::SolveAxb(vec b, vec &x) const
{
	// Solve by pivotization.
	float v00 = v[0][0];
	float v10 = v[1][0];
	float v20 = v[2][0];

	float v01 = v[0][1];
	float v11 = v[1][1];
	float v21 = v[2][1];

	float v02 = v[0][2];
	float v12 = v[1][2];
	float v22 = v[2][2];

	float av00 = glm::Abs(v00);
	float av10 = glm::Abs(v10);
	float av20 = glm::Abs(v20);

	// Find which item in first column has largest absolute value.
	if (av10 >= av00 && av10 >= av20)
	{
		Swap(v00, v10);
		Swap(v01, v11);
		Swap(v02, v12);
		Swap(b[0], b[1]);
	}
	else if (av20 >= av00)
	{
		Swap(v00, v20);
		Swap(v01, v21);
		Swap(v02, v22);
		Swap(b[0], b[2]);
	}

	/* a b c | x
	   d e f | y
	   g h i | z , where |a| >= |d| && |a| >= |g| */

	if (glm::EqualAbs(v00, 0.f))
		return false;

	// Scale row so that leading element is one.
	float denom = 1.f / v00;
//	v00 = 1.f;
	v01 *= denom;
	v02 *= denom;
	b[0] *= denom;

	/* 1 b c | x
	   d e f | y
	   g h i | z */

	// Zero first column of second and third rows.
	v11 -= v10 * v01;
	v12 -= v10 * v02;
	b[1] -= v10 * b[0];

	v21 -= v20 * v01;
	v22 -= v20 * v02;
	b[2] -= v20 * b[0];

	/* 1 b c | x
	   0 e f | y
	   0 h i | z */

	// Pivotize again.
	if (glm::Abs(v21) > glm::Abs(v11))
	{
		Swap(v11, v21);
		Swap(v12, v22);
		Swap(b[1], b[2]);
	}

	if (glm::EqualAbs(v11, 0.f))
		return false;

	/* 1 b c | x
	   0 e f | y
	   0 h i | z, where |e| >= |h| */

	denom = 1.f / v11;
//	v11 = 1.f;
	v12 *= denom;
	b[1] *= denom;

	/* 1 b c | x
	   0 1 f | y
	   0 h i | z */

	v22 -= v21 * v12;
	b[2] -= v21 * b[1];

	/* 1 b c | x
	   0 1 f | y
	   0 0 i | z */

	if (glm::EqualAbs(v22, 0.f))
		return false;

	x[2] = b[2] / v22;
	x[1] = b[1] - x[2] * v12;
	x[0] = b[0] - x[2] * v02 - x[1] * v01;

	return true;
}
