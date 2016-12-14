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

/** @file float3x3.h
	@author Jukka Jylänki
	@brief A 3-by-3 matrix for linear operations in 3D space. */
#pragma once
#ifndef FLOAT_33_H
#define FLOAT_33_H
#include "glmext.h"

/// A 3-by-3 matrix for linear transformations of 3D geometry.
/** This matrix can represent any kind of linear transformations of 3D geometry, which include rotation,
	scale, shear, mirroring and orthographic projection. A 3x3 matrix cannot represent translation (which requires
	a 3x4 matrix), or perspective projection (which requires a 4x4 matrix).

	The elements of this matrix are

		m_00, m_01, m_02
		m_10, m_11, m_12
		m_20, m_21, m_22

	The element m_yx is the value on the row y and column x.
	You can access m_yx using the double-bracket notation m[y][x], or using the member function m.At(y, x);

	@note The member functions in this class use the convention that transforms are applied to vectors in the form
	M * v. This means that "float3x3 M, M1, M2; M = M1 * M2;" gives a transformation M that applies M2 first, followed
	by M1 second, i.e. M * v = M1 * M2 * v = M1 * (M2 * v). This is the convention commonly used with OpenGL. The
	opposing convention (v * M) is commonly used with Direct3D.

	@note This class uses row-major storage, which means that the elements are packed in memory in order
	 m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], ...
	The elements for a single row of the matrix hold successive memory addresses. This is the same memory layout as
	 with C++ multidimensional arrays.

	Contrast this with column-major storage, in which the elements are packed in the memory in
	order m[0][0], m[1][0], m[2][0], m[3][0], m[0][1], m[1][1], ...
	There the elements for a single column of the matrix hold successive memory addresses.
	This is exactly opposite from the standard C++ multidimensional arrays, since if you have e.g.
	int v[10][10], then v[0][9] comes in memory right before v[1][0]. ( [0][0], [0][1], [0][2], ... [1][0], [1][1], ...) */
	class float3x3
{
public:
	/// Specifies the height of this matrix.
	enum { Rows = 3 };

	/// Specifies the width of this matrix.
	enum { Cols = 3 };

	/// Stores the data in this matrix in row-major format. [noscript]
	float v[Rows][Cols];

	/// A constant matrix that has zeroes in all its entries.
	static const float3x3 zero;

	/// A constant matrix that is the identity.
	/** The identity matrix looks like the following:
		   1 0 0
		   0 1 0
		   0 0 1
		Transforming a vector by the identity matrix is like multiplying a number by one, i.e. the vector is not changed. */
	static const float3x3 identity;

	/// A compile-time constant float3x3 which has NaN in each element.
	/// For this constant, each element has the value of quiet NaN, or Not-A-Number.
	/// @note Never compare a float3x3 to this value! Due to how IEEE floats work, "nan == nan" returns false!
	///    That is, nothing is equal to NaN, not even NaN itself!
	static const float3x3 nan;

	/// Creates a new float3x3 with uninitialized member values.
	/** [opaque-qtscript] */
	float3x3() {}

#ifdef MATH_EXPLICIT_COPYCTORS
	/// The copy-ctor for float3x3 is the trivial copy-ctor, but it is explicitly written to be able to automatically
	/// pick up this function for QtScript bindings.
	float3x3(const float3x3 &rhs) { Set(rhs); }
#endif

	/// Constructs a new float3x3 by explicitly specifying all the matrix elements.
	/// The elements are specified in row-major format, i.e. the first row first followed by the second and third row.
	/// E.g. The element _10 denotes the scalar at second (index 1) row, first (index 0) column.
	float3x3(float _00, float _01, float _02,
		float _10, float _11, float _12,
		float _20, float _21, float _22);

	/// Sets all values of this matrix.
	void Set(float _00, float _01, float _02,
		float _10, float _11, float _12,
		float _20, float _21, float _22);

	/// Sets the values of the given column.
	/** @param column The index of the column to set, in the range [0-2].
	@param data A pointer to an array of 3 floats that contain the new x, y and z values for the column. */
	void SetCol(int column, float x, float y, float z);
	void SetCol(int column, const vec &columnVector);

	/// Solves the linear equation Ax=b.
	/** The matrix A in the equations is this matrix. */
	bool SolveAxb(vec b, vec &x) const;
};
#endif
