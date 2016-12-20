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

/** @file OBB.h
	@author Jukka Jylänki
	@brief The Oriented Bounding Box (OBB) geometry object. */
#pragma once
#ifndef OBB_H
#define OBB_H

#include "Polyhedron.h"
#include "glmext.h"

/// A 3D arbitrarily oriented bounding box.
/** This data structure represents a box in 3D space. The local axes of this box can be arbitrarily oriented/rotated
	with respect to the global world coordinate system. This allows OBBs to more tightly bound objects than AABBs do,
	which always align with the world space axes. This flexibility has the drawback that the geometry tests and operations
	involving OBBs are more costly, and representing an OBB in memory takes more space (15 floats vs 6 floats). */
class OBB
{
public:
	/// The center position of this OBB.
	/** In the local space of the OBB, the center of this OBB is at (r.x,r.y,r.z), and the OBB is an AABB with
		size 2*r. */
	vec pos;

	/// Stores half-sizes to x, y and z directions in the local space of this OBB. [similarOverload: pos]
	/** These members should be positive to represent a non-degenerate OBB. */
	vec r;

	/// Specifies normalized direction vectors for the local axes. [noscript] [similarOverload: pos]
	/** axis[0] specifies the +X direction in the local space of this OBB, axis[1] the +Y direction and axis[2]
		the +Z direction.
		The scale of these vectors is always normalized. The half-length of the OBB along its local axes is
		specified by the vector r.
		The axis vectors must always be orthonormal. Be sure to guarantee that condition holds if you
		directly set to this member variable. */
	vec axis[3];

	/// The default constructor does not initialize any members of this class. [opaque-qtscript]
	/** This means that the values of the members pos, r and axis are undefined after creating a new OBB using this
		default constructor. Remember to assign to them before use.
		@see pos, r, axis. */
	OBB() {}

	/// Constructs an OBB by explicitly specifying all its member values.
	/** @see pos, r, axis. */
	OBB(const vec &pos, const vec &r, const vec &axis0, const vec &axis1, const vec &axis2);

	/// Returns the side lengths of this OBB in its local x, y and z directions.
	/** @return 2*r. */
	vec Size() const;

	/// Returns the half-side lengths of this OBB in its local x, y and z directions. [similarOverload: Size]
	/** @return r.
		@see r, Size(), HalfSize(). */
	vec HalfSize() const;

	/// Returns a diagonal vector of this OBB.
	/** This vector runs from one corner of the OBB from the opposite corner.
		@note A box has four diagonals. This function returns the direction vector from the -X-Y-Z corner to
			the +X+Y+Z corner of the OBB, in the global space of this OBB. */
	vec Diagonal() const;
	/// Returns Diagonal()/2. [similarOverload: Diagonal].
	/** @return A direction vector from the center of this OBB to the +X+Y+Z corner of this OBB, in global space.
		@see Size(), HalfSize(). */
	vec HalfDiagonal() const;
	
	/// Tests if this OBB is degenerate.
	/** @return True if this OBB does not span a strictly positive volume.
		@note This function only checks that the axis radius member of this OBB denotes a strictly positive volume, and ignores
			the position and axis direction vectors of this OBB. Be sure to check those manually for NaNs and +/-infs if that
			is desired.
		@see r, Volume(). */
	bool IsDegenerate() const;

	/// Returns the center point of this OBB in global (world) space of this OBB.
	/** @note The center point of this OBB in local space is always (r.x, r.y, r.z).
		@see pos.  */
	vec CenterPoint() const;

	/// Returns the center of mass of this OBB. [similarOverload: CenterPoint]
	/** @note This function is identical to CenterPoint(), and is provided to ease template function implementations.
		@see Volume(), SurfaceArea(). */
	vec Centroid() const { return CenterPoint(); }

	inline vec AnyPointFast() const { return pos; }

	/// Computes the volume of this OBB.
	/** @see CenterPoint(), SurfaceArea(). */
	float Volume() const;

	/// Computes the total surface area of the faces of this OBB.
	/** @see CenterPoint(), Volume(). */
	float SurfaceArea() const;

	/// Returns a corner point of this OBB.
	/** This function generates one of the eight corner points of this OBB.
		@param cornerIndex The index of the corner point to generate, in the range [0, 7].
		 The points are returned in the order 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++. (corresponding the XYZ axis directions).
		@todo Draw a diagram that shows which index generates which edge.
		@see PointInside(), Edge(), PointOnEdge(), FaceCenterPoint(), FacePoint(). */
	vec CornerPoint(int cornerIndex) const;

	/// Computes an extreme point of this OBB in the given direction.
	/** An extreme point is a farthest point of this OBB in the given direction. Given a direction,
		this point is not necessarily unique.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@return An extreme point of this OBB in the given direction. The returned point is always a
			corner point of this OBB.
		@see CornerPoint(). */
	vec ExtremePoint(const vec &direction) const;
	vec ExtremePoint(const vec &direction, float &projectionDistance) const;

	/// Projects this OBB onto the given 1D axis direction vector.
	/** This function collapses this OBB onto an 1D axis for the purposes of e.g. separate axis test computations.
		The function returns a 1D range [outMin, outMax] denoting the interval of the projection.
		@param direction The 1D axis to project to. This vector may be unnormalized, in which case the output
			of this function gets scaled by the length of this vector.
		@param outMin [out] Returns the minimum extent of this object along the projection axis.
		@param outMax [out] Returns the maximum extent of this object along the projection axis. */
	void ProjectToAxis(const vec &direction, float &outMin, float &outMax) const;

	int UniqueFaceNormals(vec *out) const;
	int UniqueEdgeDirections(vec *out) const;

	/// Returns a point on an edge of this OBB.
	/** @param edgeIndex The index of the edge to generate a point to, in the range [0, 11]. @todo Document which index generates which one.
		@param u A normalized value between [0,1]. This specifies the relative distance of the point along the edge.
		@see PointInside(), Edge(), CornerPoint(), FaceCenterPoint(), FacePoint(). */
	vec PointOnEdge(int edgeIndex, float u) const;

	/// Returns the point at the center of the given face of this OBB.
	/** @param faceIndex The index of the OBB face to generate the point at. The valid range is [0, 5].
		@todo Document which index generates which face.
		@see PointInside(), Edge(), CornerPoint(), PointOnEdge(), FacePoint(). */
	vec FaceCenterPoint(int faceIndex) const;

	/// Generates a point at the surface of the given face of this OBB.
	/** @param faceIndex The index of the OBB face to generate the point at. The valid range is [0, 5].
		@param u A normalized value between [0, 1].
		@param v A normalized value between [0, 1].
		@todo Document which index generates which face.
		@see PointInside(), Edge(), CornerPoint(), PointOnEdge(), FaceCenterPoint(), FacePlane(). */
	vec FacePoint(int faceIndex, float u, float v) const;

	/// Fills an array with all the eight corner points of this OBB.
	/** @param outPointArray [out] The array to write the points to. Must have space for 8 elements.
		@see CornerPoint(), GetFacePlanes(). */
	void GetCornerPoints(vec *outPointArray) const;

	/// Finds the two extreme points along the given direction vector from the given point array.
	/** @param dir The direction vector to project the point array to. This vector does not need to be normalized.
		@param pointArray [in] The list of points to process.
		@param numPoints The number of elements in pointArray.
		@param idxSmallest [out] The index of the smallest point along the given direction will be received here.
			This pointer may be left null, if this information is of no interest.
		@param idxLargest [out] The index of the largest point along the given direction will be received here.
			This pointer may be left null, if this information is of no interest. */
	static void ExtremePointsAlongDirection(const vec &dir, const vec *pointArray, int numPoints, int &idxSmallest, int &idxLargest) { float smallestD, largestD; ExtremePointsAlongDirection(dir, pointArray, numPoints, idxSmallest, idxLargest, smallestD, largestD); }
	/** @param smallestD [out] Receives the minimum projection distance along the given direction.
		@param largestD [out] Receives the maximum projection distance along the given direction. */
	static void ExtremePointsAlongDirection(const vec &dir, const vec *pointArray, int numPoints, int &idxSmallest, int &idxLargest, float &smallestD, float &largestD);

	/// Computes the smallest OBB by volume that encloses the given point set.
	/** This function implements the algorithm from the paper
		An Exact Algorithm for Finding Minimum Oriented Bounding Boxes, Jukka Jylänki, 2015. Available at http://clb.demon.fi/minobb/ */
	static OBB OptimalEnclosingOBB(const vec *pointArray, int numPoints, bool gpu);
	static OBB OptimalEnclosingOBB(const Polyhedron &convexPolyhedron, bool gpu);
};

#endif