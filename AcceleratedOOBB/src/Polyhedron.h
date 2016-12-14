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

/** @file Polyhedron.h
	@author Jukka Jylänki
	@brief The Polyhedron geometry object. */
#pragma once
#ifndef POLYHEDRON_H
#define POLYHEDRON_H

//#ifdef MATH_ENABLE_STL_SUPPORT
#include <vector>
#include <string>
#include "glmext.h"
#include "Plane.h"

//#endif

/// Represents a three-dimensional closed geometric solid defined by flat polygonal faces.
class Polyhedron
{
public:
	/// Stores a list of indices of a single face of a Polyhedron.
	struct Face
	{
		/// Specifies the indices of the corner vertices of this polyhedron.
		/// Indices point to the polyhedron vertex array.
		/// The face vertices should all lie on the same plane.
		/// The positive direction of the plane (the direction the face outwards normal points to)
		/// is the one where the vertices are wound in counter-clockwise order.
		std::vector<int> v;

		/// Reverses the winding order of this face. This has the effect of reversing the direction
		/// the normal of this face points to.
		void FlipWindingOrder();

		/// Returns a string of form "0,1,2,3,4" that refers to the indices of the vertices that this face uses.
		std::string ToString() const;

		static Face FromString(const char *str);
	};

	/// Specifies the vertices of this polyhedron.
	VecArray v;

	/// Specifies the individual faces of this polyhedron.  [similarOverload: v]
	/** Each face is described by a list of indices to the vertex array. The indices define a
		simple polygon in counter-clockwise winding order. */
	std::vector<Face> f;

	/// Returns the number of vertices in this polyhedron.
	/** The running time of this function is O(1).
	@see NumFaces(), NumEdges(), EulerFormulaHolds(). */
	int NumVertices() const { return (int)v.size(); }

	/// Returns the number of faces in this polyhedron.
	/** The running time of this function is O(1).
	@see NumVertices(), NumEdges(), EulerFormulaHolds(), FacePolygon(), FacePlane(). */
	int NumFaces() const { return (int)f.size(); }

	/// Returns the number of (unique) edges in this polyhedron.
	/** This function will enumerate through all faces of this polyhedron to compute the number of unique edges.
	The running time is linear to the number of faces and vertices in this Polyhedron.
	@see NumVertices(), NumFaces(), EulerFormulaHolds(), Edge(), Edges(), EdgeIndices(). */
	int NumEdges() const;

	/// The default constructor creates a null polyhedron.
	/** A null polyhedron has 0 vertices and 0 faces.
		@see IsNull(). */
	Polyhedron() {}
	
	// Computes the most extreme point of this convex Polyhedron into the given direction.
	/** @param adjacencyData A precomputed data structure that specifies the adjacency information between the vertices of this Polyhedron.
			Call GenerateVertexAdjacencyData() to compute this structure.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@param floodFillVisited A temporary structure to an array of size |V| where each element specifies whether the extreme
			vertex search has visited that vertex or not. If floodFillVisited[i] == floodFillVisitColor, then the vertex i
			has been visited, otherwise not.
		@param mostExtremeDistance [out] Receives the 1D projection distance of the most extreme vertex onto the direction vector.
		@param startingVertex [optional] Specifies a hint vertex from where to start the search. Specifying a know vertex that is close
			to being the most extreme vertex in the given direction may speed up the search.
		@return The index of the most extreme vertex into the specified direction. */

#define MATH_NUMSTEPS_STATS

#ifdef MATH_NUMSTEPS_STATS
	mutable int numSearchStepsDone, numImprovementsMade;
#endif
	int ExtremeVertexConvex(const std::vector<std::vector<int> > &adjacencyData, const vec &direction,
		std::vector<unsigned int> &floodFillVisited, unsigned int floodFillVisitColor, float &mostExtremeDistance, int startingVertex = 0) const;

	/// Computes a data structure that specifies adjacent vertices for each vertex.
	/** In the returned vector of vectors V, the vector V[i] specifies all the vertex indices that vertex i
		is connected to. */
	std::vector<std::vector<int> > GenerateVertexAdjacencyData() const;

	/// Returns the plane of the given polyhedron face.
	/** The normal of the plane points outwards from this polyhedron, i.e. towards the space that
	is not part of the polyhedron.
	This function assumes that the given face of the polyhedron is planar, as should be for all
	well-formed polyhedron.
	@param faceIndex The index of the face to get, in the range [0, NumFaces()-1].
	@see NumFaces(), FacePolygon(). */
	Plane FacePlane(int faceIndex) const;

	/// Returns the normalized normal vector of the given face.
	vec FaceNormal(int faceIndex) const;

	/// Creates a Polyhedron object that represents the convex hull of the given point array.
	/// \todo This function is strongly WIP!
	static Polyhedron ConvexHull(const VecArray &points) { return !points.empty() ? ConvexHull((const vec*)&points[0], (int)points.size()) : Polyhedron(); }
	static Polyhedron ConvexHull(const vec *pointArray, int numPoints);

	void RemoveDegenerateFaces();
	void RemoveRedundantVertices();
};

#endif
