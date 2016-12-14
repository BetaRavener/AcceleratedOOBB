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

/** @file Plane.h
	@author Jukka Jylänki
	@brief The Plane geometry object. */
#pragma once
#ifndef PLANE_H
#define PLANE_H

#include "Line.h"
#include "glmext.h"

/// Specifies a plane in 3D space. This plane is an affine 2D subspace of the 3D space, meaning
/// that its sides extend to infinity, and it does not necessarily pass through the origin.
class Plane
{
public:
	/// The direction this plane is facing at.
	/** This direction vector is always normalized. If you assign to this directly, please remember to only
		assign normalized direction vectors. */
	vec normal;
	/// The offset of this plane from the origin. [similarOverload: normal]
	/** The value -d gives the signed distance of this plane from origin.
		Denoting normal:=(a,b,c), this class uses the convention ax+by+cz = d, which means that:
		 - If this variable is positive, the vector space origin (0,0,0) is on the negative side of this plane.
		 - If this variable is negative, the vector space origin (0,0,0) is on the on the positive side of this plane.
		@note Some sources use the opposite convention ax+by+cz+d = 0 to define the variable d. When comparing equations
			and algorithm regarding planes, always make sure you know which convention is being used, since it affects the
			sign of d. */
	float d;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members normal and d are undefined after creating a new Plane using this
		default constructor. Remember to assign to them before use.
		@see normal, d. */
	Plane() {}
	/// Constructs a plane by directly specifying the normal and distance parameters.
	/** @param normal The direction the plane is facing. This vector must have been normalized in advance.
		@param d The offset of this plane from the origin. The value -d gives the signed distance of this plane from the origin.
		@see normal, d. */
	Plane(const vec &normal, float d);
	/// Constructs a plane by specifying three points on the plane.
	/** The normal of the plane will point to
		the halfspace from which the points are observed to be oriented in counter-clockwise order.
		@note The points v1, v2 and v3 must not all lie on the same line.
		@see Set(). */
	Plane(const vec &v1, const vec &v2, const vec &v3);
	/// Constructs a plane by specifying a single point on the plane, and the surface normal.
	/** @param normal The direction the plane is facing. This vector must have been normalized in advance.
		@see Set(). */
	Plane(const vec &point, const vec &normal);
	/// Constructs a plane by specifying a line that lies on the plane, and the plane normal.
	/** @param line The line object that is to be contained in the newly constructed plane.
		@param normal The direction the plane if facing. This vector must have been normalized in advance. The normal
			of the line must not be collinear with the direction of this normal. If a line segment is specified, the line
			segment must not be degenerate. */
	Plane(const Line &line, const vec &normal);
	void Set(const vec& v1, const vec& v2, const vec& v3);
	void Set(const vec& point, const vec& normal_);
	/// Returns the signed distance of this plane to the given point.
	/** If this function returns a negative value, the given point lies in the negative halfspace of this plane.
		Conversely, if a positive value is returned, then the given point lies in the positive halfspace of this plane.
		@see Distance(), IsOnPositiveSide(), AreOnSameSide(). */
	float SignedDistance(const vec &point) const;
	float Distance(const vec &point) const;
	bool Contains(const vec &point, float epsilon = 1e-3f) const;
};
#endif
