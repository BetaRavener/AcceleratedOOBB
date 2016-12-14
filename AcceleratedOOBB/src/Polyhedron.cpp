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

/** @file Polyhedron.cpp
	@author Jukka Jylänki
	@brief Implementation for the Polyhedron geometry object. */
#include "Polyhedron.h"
#include <set>
#include <map>
#include <utility>
#include <list>
#include <sstream>
#include <stdlib.h>
#include "OBB.h"
#include "glmext.h"
#include <random>
#include "Plane.h"
#include "Line.h"

#if __cplusplus > 199711L // Is C++11 or newer?
#define HAS_UNORDERED_MAP
#endif

#ifdef HAS_UNORDERED_MAP
#include <unordered_map>
#else
#include <map>
#endif

#define FLOAT_INF INFINITY
#define DIR_VEC vec
#define assume ((void)0)
#define MARK_UNUSED ((void)0)
#define FORCE_INLINE __inline
#define DIR_TO_FLOAT4(v) (cv(v))
#define POINT_TO_FLOAT4(v) (cv(v))

#ifndef ARRAY_LENGTH
#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))
#endif

typedef glm::dvec3 cv;
typedef double cs;
typedef std::vector<glm::dvec3> VecdArray;

void Polyhedron::Face::FlipWindingOrder()
{
	for (size_t i = 0; i < v.size() / 2; ++i)
		Swap(v[i], v[v.size() - 1 - i]);
}

std::string Polyhedron::Face::ToString() const
{
	std::stringstream ss;
	for(size_t i = 0; i < v.size(); ++i)
		ss << v[i] << ((i!=v.size()-1) ? ", " : "");
	return ss.str();
}

Polyhedron::Face Polyhedron::Face::FromString(const char *str)
{
	Face f;
	if (!str)
		return f;
	while(*str)
	{
		char *endptr = 0;
		int idx = (int)strtol(str, &endptr, 10);
		str = endptr;
		while(*str == ',' || *str == ' ')
			++str;
		f.v.push_back(idx);
	}
	return f;
}

int Polyhedron::NumEdges() const
{
	int numEdges = 0;
	for (size_t i = 0; i < f.size(); ++i)
		numEdges += (int)f[i].v.size();
	return numEdges / 2;
}



int Polyhedron::ExtremeVertexConvex(const std::vector<std::vector<int> > &adjacencyData, const vec &direction, 
	std::vector<unsigned int> &floodFillVisited, unsigned int floodFillVisitColor,
	float &mostExtremeDistance, int startingVertex) const
{
#ifdef MATH_NUMSTEPS_STATS
	numSearchStepsDone = 0;
	numImprovementsMade = 0;
#endif
	float curD = direction.Dot(this->v[startingVertex]);
	const int *neighbors = &adjacencyData[startingVertex][0];
	const int *neighborsEnd = neighbors + adjacencyData[startingVertex].size();
	floodFillVisited[startingVertex] = floodFillVisitColor;

	int secondBest = -1;
	float secondBestD = curD - 1e-3f;
	while(neighbors != neighborsEnd)
	{
#ifdef MATH_NUMSTEPS_STATS
		++numSearchStepsDone;
#endif
		int n = *neighbors++;
		if (floodFillVisited[n] != floodFillVisitColor)
		{
			float d = direction.Dot(this->v[n]);
			if (d > curD)
			{
#ifdef MATH_NUMSTEPS_STATS
				++numImprovementsMade;
#endif
				startingVertex = n;
				curD = d;
				floodFillVisited[startingVertex] = floodFillVisitColor;
				neighbors = &adjacencyData[startingVertex][0];
				neighborsEnd = neighbors + adjacencyData[startingVertex].size();
				secondBest = -1;
				secondBestD = curD - 1e-3f;
			}
			else if (d > secondBestD)
			{
				secondBest = n;
				secondBestD = d;
			}
		}
	}
	if (secondBest != -1 && floodFillVisited[secondBest] != floodFillVisitColor)
	{
		float secondMostExtreme = -FLOAT_INF;
#ifdef MATH_NUMSTEPS_STATS
		int numSearchStepsDoneParent = numSearchStepsDone;
		int numImprovementsMadeParent = numImprovementsMade;
#endif
		int secondTry = ExtremeVertexConvex(adjacencyData, direction, floodFillVisited, floodFillVisitColor, secondMostExtreme, secondBest);
#ifdef MATH_NUMSTEPS_STATS
		numSearchStepsDone += numSearchStepsDoneParent;
		numImprovementsMade += numImprovementsMadeParent;
#endif
		if (secondMostExtreme > curD)
		{
			mostExtremeDistance = secondMostExtreme;
			return secondTry;
		}
	}
	mostExtremeDistance = curD;
	return startingVertex;
}

std::vector<std::vector<int>> Polyhedron::GenerateVertexAdjacencyData() const
{
	std::vector<std::vector<int> > adjacencyData;
	adjacencyData.reserve(v.size());
	adjacencyData.insert(adjacencyData.end(), v.size(), std::vector<int>());
	for (size_t i = 0; i < f.size(); ++i)
	{
		const Face &face = f[i];
		int v0 = face.v.back();
		for (size_t j = 0; j < face.v.size(); ++j)
		{
			int v1 = face.v[j];
			adjacencyData[v0].push_back(v1);
			v0 = v1;
		}
	}
	return adjacencyData;
}

Plane Polyhedron::FacePlane(int faceIndex) const
{
	const Face &face = f[faceIndex];
	if (face.v.size() >= 3)
		return Plane(v[face.v[0]], v[face.v[1]], v[face.v[2]]);
	else if (face.v.size() == 2)
		return Plane(Line(v[face.v[0]], v[face.v[1]]), vec((vec)v[face.v[0]] - (vec)v[face.v[1]]).Perpendicular());
	else if (face.v.size() == 1)
		return Plane(v[face.v[0]], DIR_VEC(0, 1, 0));
	else
		return Plane();
}

cv PolyFaceNormal(const Polyhedron &poly, int faceIndex)
{
	const Polyhedron::Face &face = poly.f[faceIndex];
	if (face.v.size() == 3)
	{
		cv a = DIR_TO_FLOAT4(poly.v[face.v[0]]);
		cv b = DIR_TO_FLOAT4(poly.v[face.v[1]]);
		cv c = DIR_TO_FLOAT4(poly.v[face.v[2]]);
		cv normal = glm::normalize(glm::cross((b - a), (c - a)));
		return normal;
		//		return ((vec)v[face.v[1]]-(vec)v[face.v[0]]).Cross((vec)v[face.v[2]]-(vec)v[face.v[0]]).Normalized();
	}
	else if (face.v.size() > 3)
	{
		// Use Newell's method of computing the face normal for best stability.
		// See Christer Ericson, Real-Time Collision Detection, pp. 491-495.
		cv normal(0, 0, 0);
		int v0 = face.v.back();
		for (size_t i = 0; i < face.v.size(); ++i)
		{
			int v1 = face.v[i];
			normal.x += (cs(poly.v[v0].y) - poly.v[v1].y) * (cs(poly.v[v0].z) + poly.v[v1].z); // Project on yz
			normal.y += (cs(poly.v[v0].z) - poly.v[v1].z) * (cs(poly.v[v0].x) + poly.v[v1].x); // Project on xz
			normal.z += (cs(poly.v[v0].x) - poly.v[v1].x) * (cs(poly.v[v0].y) + poly.v[v1].y); // Project on xy
			v0 = v1;
		}
		normal = glm::normalize(normal);
		return normal;
	}
	else if (face.v.size() == 2)
		return DIR_TO_FLOAT4(glm::normalize(glm::cross(((vec)poly.v[face.v[1]] - (vec)poly.v[face.v[0]]), vec((vec)poly.v[face.v[0]] - (vec)poly.v[face.v[1]]).Perpendicular() - poly.v[face.v[0]])));
	else if (face.v.size() == 1)
		return cv(0, 1, 0);
	else
		return cv(NAN, NAN, NAN);
}

vec Polyhedron::FaceNormal(int faceIndex) const
{
	cv normal = PolyFaceNormal(*this, faceIndex);
	return DIR_VEC((float)normal.x, (float)normal.y, (float)normal.z);
}

/// Edge from v1->v2.
struct AdjEdge
{
//	int v1;
//	int v2;
	int f1; // The face that has v1->v2.
	int f2; // The face that has v2->v1.
};

#include <list>

struct CHullHelp
{
	std::map<std::pair<int,int>, AdjEdge> edges;
	std::list<int> livePlanes;
};

namespace
{
	struct hash_edge
	{
		size_t operator()(const std::pair<int, int> &e) const
		{
			return (e.first << 16) ^ e.second;
		}
	};
}

#if 0
static bool ContainsAndRemove(std::vector<int> &arr, int val)
{
	for(size_t i = 0; i < arr.size(); ++i)
		if (arr[i] == val)
		{
			arr.erase(arr.begin() + i);
			return true;
		}
	return false;
}
#endif

#define LEX_ORDER(x, y) if ((x) < (y)) return -1; else if ((x) > (y)) return 1;
int LexFloat3Cmp(const vec &a, const vec &b)
{
	LEX_ORDER(a.x, b.x);
	return 0;
}


int LexFloat3CmpV(const void *a, const void *b) { return LexFloat3Cmp(*reinterpret_cast<const vec*>(a), *reinterpret_cast<const vec*>(b)); }


Polyhedron Polyhedron::ConvexHull(const vec *pointArray, int numPoints)
{
	std::set<int> extremes;
	std::linear_congruential_engine<unsigned int, 69621, 0, 0x7FFFFFFF> eng;
	eng.seed(123);

	Polyhedron p;

	const cv dirs[] =
	{
		cv(-1, -1, -1),
		cv(1, 0, 0),
		cv(0, 1, 0),
		cv(0, 0, 1),

		cv(1, 1, 0), cv(1, 0, 1), cv(0, 1, 1),
		cv(1, -1, 0), cv(1, 0, -1), cv(0, 1, -1),
		cv(1, 1, 1), cv(-1, 1, 1), cv(1, -1, 1),
		cv(1, 1, -1)
	};

	for(size_t i = 0; i < ARRAY_LENGTH(dirs) && extremes.size() < 4; ++i)
	{
		int extremeI = 0;
		cs largestD = -FLOAT_INF;
		for(int j = 0; j < numPoints; ++j)
		{
			cs d = glm::dot(dirs[i], DIR_TO_FLOAT4(pointArray[j]));
			if (d > largestD)
			{
				largestD = d;
				extremeI = j;
			}
		}
		extremes.insert(extremeI);
	}

//	assume(extremes.size() >= 3);
	if (extremes.size() < 3)
		return p; // This might happen if there's NaNs in the vertex data, or duplicates.

	// Handle degenerate case when the predefined directions did not find a nonzero volume.
	if (extremes.size() == 3)
	{
		std::set<int>::iterator iter = extremes.begin();
		int v0 = *iter++;
		int v1 = *iter++;
		int v2 = *iter;
		Plane plane(pointArray[v0], pointArray[v1], pointArray[v2]);
		for(int i = 0; i < numPoints; ++i)
		{
			if (!plane.Contains(pointArray[i]))
				extremes.insert(i);
			if (extremes.size() >= 4)
				break;
		}

		// The degenerate case when all vertices in the input data set are planar.
		if (extremes.size() == 3)
		{
			p.v.push_back(pointArray[v0]);
			p.v.push_back(pointArray[v1]);
			p.v.push_back(pointArray[v2]);

			Face f;
			f.v.push_back(0);
			f.v.push_back(1);
			f.v.push_back(2);
			p.f.push_back(f);
			f.v[0] = 2;
			f.v[2] = 0;
			p.f.push_back(f);
			return p;
		}
	}

	p.v.insert(p.v.end(), pointArray, pointArray + numPoints);

	{
		std::set<int>::iterator iter = extremes.begin();
		int v0 = *iter; ++iter;
		int v1 = *iter; ++iter;
		int v2 = *iter; ++iter;
		int v3 = *iter;
		assert(v0 < v1 && v1 < v2 && v2 < v3);
		Swap(p.v[0], p.v[v0]);
		Swap(p.v[1], p.v[v1]);
		Swap(p.v[2], p.v[v2]);
		Swap(p.v[3], p.v[v3]);
	}

	// For each face, maintain a list of its adjacent faces.
//	std::vector<std::vector<int> > faceAdjacency(4);
	// For each face, precompute its normal vector.
	VecdArray faceNormals(4);

	Face face;
	face.v.resize(3);
	face.v[0] = 0; face.v[1] = 1; face.v[2] = 2; p.f.push_back(face);
	face.v[0] = 0; face.v[1] = 1; face.v[2] = 3; p.f.push_back(face);
	face.v[0] = 0; face.v[1] = 2; face.v[2] = 3; p.f.push_back(face);
	face.v[0] = 1; face.v[1] = 2; face.v[2] = 3; p.f.push_back(face);

	// Ensure that the winding order of the generated tetrahedron is correct for each face.
	if (p.FacePlane(0).SignedDistance(p.v[3]) > 0.f) p.f[0].FlipWindingOrder();
	if (p.FacePlane(1).SignedDistance(p.v[2]) > 0.f) p.f[1].FlipWindingOrder();
	if (p.FacePlane(2).SignedDistance(p.v[1]) > 0.f) p.f[2].FlipWindingOrder();
	if (p.FacePlane(3).SignedDistance(p.v[0]) > 0.f) p.f[3].FlipWindingOrder();

#ifdef CONVEXHULL_VERBOSE
	p.DumpStructure();
#endif

//	faceAdjacency[0].push_back(1); faceAdjacency[0].push_back(2); faceAdjacency[0].push_back(3);
//	faceAdjacency[1].push_back(2); faceAdjacency[1].push_back(3); faceAdjacency[1].push_back(0);
//	faceAdjacency[2].push_back(1); faceAdjacency[2].push_back(3); faceAdjacency[2].push_back(0);
//	faceAdjacency[3].push_back(1); faceAdjacency[3].push_back(2); faceAdjacency[3].push_back(0);
	faceNormals[0] = cv(DIR_TO_FLOAT4(p.FaceNormal(0)));
	faceNormals[1] = cv(DIR_TO_FLOAT4(p.FaceNormal(1)));
	faceNormals[2] = cv(DIR_TO_FLOAT4(p.FaceNormal(2)));
	faceNormals[3] = cv(DIR_TO_FLOAT4(p.FaceNormal(3)));

#ifdef HAS_UNORDERED_MAP
	std::unordered_map<std::pair<int, int>, int, hash_edge> edgesToFaces;
#else
	std::map<std::pair<int, int>, int> edgesToFaces;
#endif
	for(size_t i = 0; i < p.f.size(); ++i)
	{
		const Polyhedron::Face &f = p.f[i];
		int v0 = f.v.back();
		for(size_t j = 0; j < f.v.size(); ++j)
		{
			int v1 = f.v[j];
			edgesToFaces[std::make_pair(v0, v1)] = (int)i;
			v0 = v1;
		}
	}

	// If a vertex of the input point set is on the positive side of a face of the partially built convex hull, we
	// call the vertex to be in conflict with that face, because due to the position of that vertex, the given face cannot
	// be a face of the final convex hull.

	// For each face of the partial convex hull, maintain a 'conflict list'.
	// The conflict list represents for each face of the so far built convex hull the list of vertices that conflict
	// with that face. The list is not complete in the sense that a vertex is only listed with one (arbitrary) face that it
	// conflicts with, and not all of them.
	std::vector<std::vector<int> > conflictList(p.f.size());

	// For each vertex, maintain a conflict list of faces as well.
	std::vector<std::set<int> > conflictListVertices(p.v.size());

#ifdef MATH_CONVEXHULL_DOUBLE_PRECISION
	const double inPlaneEpsilon = 1e-5;
#else
	const float inPlaneEpsilon = 1e-4f;
#endif

	// Assign each remaining vertex (vertices 0-3 form the initial hull) to the initial conflict lists.
	for(size_t j = 0; j < p.f.size(); ++j)
	{
		cv pointOnFace = POINT_TO_FLOAT4(p.v[p.f[j].v[0]]);
		for(size_t i = 4; i < p.v.size(); ++i)
		{
			cs d = glm::dot(cv(faceNormals[j]), cv(POINT_TO_FLOAT4(p.v[i])) - pointOnFace);
			if (d > inPlaneEpsilon)
			{
				conflictList[j].push_back((int)i);
				conflictListVertices[i].insert((int)j);
			}
		}
	}

	std::vector<int> workStack;
	if (!conflictList[0].empty()) workStack.push_back(0);
	if (!conflictList[1].empty()) workStack.push_back(1);
	if (!conflictList[2].empty()) workStack.push_back(2);
	if (!conflictList[3].empty()) workStack.push_back(3);

	std::set<int> conflictingVertices;
	std::vector<std::pair<int, int> > boundaryEdges;
	std::vector<int> faceVisitStack;

	std::vector<int> hullVertices(4, 1);

	// We need to perform flood fill searches across the faces to scan the interior faces vs border faces, so maintain
	// an auxiliary data structure to store already visited faces.
	std::vector<int> floodFillVisited(p.f.size());
	int floodFillVisitColor = 1;
//	p.DumpStructure();

#ifdef CONVEXHULL_VERBOSE
	for(size_t j = 0; j < p.f.size(); ++j)
		if (!p.f[j].v.empty())
		{
			vec pointOnFace = p.v[p.f[j].v[0]];
			for(size_t i = 0; i < p.v.size(); ++i)
			{
				float d = Dot((vec)p.v[i] - pointOnFace, faceNormals[j]);
				if (d > inPlaneEpsilon)
					LOG(MathLogWarningNoCallstack, "Vertex %d is at distance %f from face %d.", (int)i, d, (int)j);
//				else
//					LOGI("Vertex %d is at distance %f from face %d.", (int)i, d, (int)j);
			}
		}
#endif

	while(!workStack.empty())
	{
		// Choose a random plane in order to avoid a degenerate worst case processing.
		std::uniform_int_distribution<> rngRange(0, (int)workStack.size() - 1); // define the range
		int fIdx = rngRange(eng);
//		int f = workStack[fIdx];
		int f = workStack.at(fIdx);
		Swap(workStack[fIdx], workStack.back());
		workStack.pop_back();
		std::vector<int> &conflict = conflictList.at(f);
//		std::vector<int> &conflict = conflictList[f];
		if (conflict.empty())
			continue;

		cv pointOnFace = POINT_TO_FLOAT4(p.v.at(p.f.at(f).v.at(0)));
//		vec pointOnFace = p.v[p.f[f].v[0]];
		// Find the most extreme conflicting vertex on this face.
		cs extremeD = -FLOAT_INF;
		int extremeCI = -1; // Index of the vertex in the conflict list.
		int extremeI = -1; // Index of the vertex in the convex hull.
		for(size_t i = 0; i < conflict.size(); ++i)
		{
			int vt = conflict.at(i);
//			int vt = conflict[i];
			if (vt < (int)hullVertices.size() && hullVertices[vt])
				continue; // Robustness check: if this vertex is already part of the hull, ignore it.
			cs d = glm::dot(cv(faceNormals[f]), cv(POINT_TO_FLOAT4(p.v[vt])) - pointOnFace);
			if (d > extremeD)
			{
				extremeD = d;
				extremeCI = (int)i;
				extremeI = vt;
			}
		}
		// Remove the most extreme conflicting vertex from the conflict list, because
		// that vertex will become a part of the convex hull.
		if (extremeCI == -1)
		{
			conflict.clear();
			continue;
		}
		Swap(conflict.at(extremeCI), conflict.back());
//		Swap(conflict[extremeCI], conflict.back());
		conflict.pop_back();

//		if (extremeD <= ((Plane)facePlanes[f]).d + 1e-5f)
		if (extremeD <= inPlaneEpsilon)
			continue;

		std::set<int> &conflictingFaces = conflictListVertices[extremeI];

//		floodFillVisited[f] = floodFillVisitColor;
		floodFillVisited.at(f) = floodFillVisitColor;
		faceVisitStack.push_back(f);
		faceVisitStack.insert(faceVisitStack.end(), conflictingFaces.begin(), conflictingFaces.end());
		for(std::set<int>::iterator iter = conflictingFaces.begin(); iter != conflictingFaces.end(); ++iter)
			floodFillVisited.at(*iter) = floodFillVisitColor;

		while(!faceVisitStack.empty())
//		for(std::set<int>::iterator iter = conflictingFaces.begin();
//			iter != conflictingFaces.end(); ++iter)
		{
			int fi = faceVisitStack.back();
			faceVisitStack.pop_back();
			conflictingVertices.insert(conflictList[fi].begin(), conflictList[fi].end());
			conflictList.at(fi).clear();
//			conflictList[fi].clear();

			// Traverse through each edge of this face to detect whether this is an interior or a boundary face.
			const Polyhedron::Face &pf = p.f.at(fi);

			if (pf.v.empty())
				continue;

//			const Polyhedron::Face &f = p.f[fi];
			int v0 = pf.v.back();
			for(size_t j = 0; j < pf.v.size(); ++j)
			{
					int v1 = pf.v[j];
				int adjFace = edgesToFaces[std::make_pair(v1, v0)];
#ifdef CONVEXHULL_VERBOSE
				p.DumpStructure();
#endif
				if (adjFace == -1 || p.f[adjFace].v.empty() || floodFillVisited[adjFace] == floodFillVisitColor)
				{
					v0 = v1;
					continue; // The face does not exist anymore, or we have already visited it.
				}

				assert(edgesToFaces[std::make_pair(v0, v1)] == fi);
				assert(adjFace != fi);

				bool adjFaceIsInConflict = (conflictingFaces.find(adjFace) != conflictingFaces.end());

//				LOGI("Edge %d->%d is adjacent face %d", v1, v0, adjFace);
//				if (!p.f[adjFace].v.empty())
//				{
					cs d;
					cv ptOnFace = POINT_TO_FLOAT4(p.v[p.f[adjFace].v[0]]);
					d = glm::dot(cv(faceNormals[adjFace]), cv(POINT_TO_FLOAT4(p.v[extremeI])) - ptOnFace);
//					if (((Plane)facePlanes[adjFace]).SignedDistance(p.v[extremeI]) > 1e-4f) // Is v0<->v1 an interior edge?
//					bool containsVtx = ContainsAndRemove(conflictList[adjFace], extremeI);
					if (d > inPlaneEpsilon)
						adjFaceIsInConflict = true;
//				}
				if (adjFaceIsInConflict)
				{
					if (floodFillVisited[adjFace] != floodFillVisitColor) // Add the neighboring face to the visit stack.
					{
						faceVisitStack.push_back(adjFace);
//							floodFillVisited[adjFace] = floodFillVisitColor;
						floodFillVisited.at(adjFace) = floodFillVisitColor;
					}
				}
				else // v0<->v1 is a boundary edge.
				{
					boundaryEdges.push_back(std::make_pair(v0, v1));
				}
				v0 = v1;
			}

			// Mark this face as deleted by setting its size to zero vertices. This is better than erasing the face immediately,
			// as that incurs memory allocation and bookkeeping costs. The null faces are all removed at the very end in one pass.
			//p.f[fi].v.clear();
			int w0 = p.f.at(fi).v.back();
			for(size_t i = 0; i < p.f[fi].v.size(); ++i)
			{
				int w1 = p.f[fi].v[i];
				edgesToFaces[std::make_pair(w0, w1)] = -1;
				w0 = w1;
			}
			p.f.at(fi).v.clear();
		}
		++floodFillVisitColor;

		// Since we deleted a bunch of faces, remove those faces from the conflict lists of each vertex.
		for(std::set<int>::iterator vi = conflictingVertices.begin(); vi != conflictingVertices.end(); ++vi)
		{
			int v = *vi;
			if (v != extremeI)
			{
				for(std::set<int>::iterator fi = conflictingFaces.begin(); fi != conflictingFaces.end(); ++fi)
				{
					std::set<int>::iterator iter = conflictListVertices[v].find(*fi);
					//assert(iter3 != conflictListVertices[*iter].end());
					if (iter != conflictListVertices[v].end())
					{
						conflictListVertices[v].erase(iter);
					}
				}
			}
		}

		conflictingFaces.clear();

		// Reconstruct the proper CCW order of the boundary. Note that it is possible to perform a search where the order
		// would come out right from the above graph search without needing to sort, perhaps a todo for later.
		for(size_t i = 0; i < boundaryEdges.size(); ++i)
			for(size_t j = i+1; j < boundaryEdges.size(); ++j)
				if (boundaryEdges[i].second == boundaryEdges[j].first)
				{
					Swap(boundaryEdges[i+1], boundaryEdges[j]);
					break;
				}
#ifdef CONVEXHULL_VERBOSE
		LOGI("New boundary:");
		for(size_t i = 0; i < boundaryEdges.size(); ++i)
			LOGI("%d->%d", (int)boundaryEdges[i].first, boundaryEdges[i].second);
#endif

		std::pair<int, int> prev = boundaryEdges.back();
		for(size_t i = 0; i < boundaryEdges.size(); ++i)
		{
			if (prev.second != boundaryEdges[i].first)
			{
				assert(false);
				return Polyhedron();
			}
			prev = boundaryEdges[i];
		}

#if 0
		std::vector<Tri> degenerateTris;
#endif

		size_t oldNumFaces = p.f.size();
		// Create new faces to close the boundary.
		for(size_t i = 0; i < boundaryEdges.size(); ++i)
		{
			assert(face.v.size() == 3);
			face.v[0] = boundaryEdges[i].first; face.v[1] = boundaryEdges[i].second; face.v[2] = extremeI; p.f.push_back(face);

#if 0
			// Test the dimensions of the new face.
			cv a = POINT_TO_FLOAT4(p.v[face.v[0]]);
			cv b = POINT_TO_FLOAT4(p.v[face.v[1]]);
			cv c = POINT_TO_FLOAT4(p.v[face.v[2]]);
			if (a.DistanceSq(b) < 1e-7f || a.DistanceSq(c) < 1e-7f || b.DistanceSq(c) < 1e-7f)
				LOGW("Creating a degenerate face!");
#endif
			//vec faceNormal = p.FaceNormal(p.f.size()-1);
			//cv faceNormal = (b-a).Cross(c-a);
			//cs len = faceNormal.Normalize();
			cv faceNormal = PolyFaceNormal(p, (int)p.f.size()-1);
#if 0
			if (len < 1e-3f || a.DistanceSq(b) < 1e-7f || a.DistanceSq(c) < 1e-7f || b.DistanceSq(c) < 1e-7f)
			{
				LOGW("Face has degenerate vertices %s, %s, %s! (normal was of len %f)", vec(p.v[face.v[0]]).ToString().c_str(), vec(p.v[face.v[1]]).ToString().c_str(), vec(p.v[face.v[2]]).ToString().c_str(), len);
				Tri t;
				t.v[0] = face.v[0];
				t.v[1] = face.v[1];
				t.v[2] = face.v[2];
				t.faceIndex = (int)p.f.size()-1;
				degenerateTris.push_back(t);
			}
#endif
			faceNormals.push_back(faceNormal);
			assert(extremeI >= (int)hullVertices.size() || !hullVertices[extremeI]);
			assert(edgesToFaces.find(std::make_pair(boundaryEdges[i].first, boundaryEdges[i].second))
				== edgesToFaces.end() || edgesToFaces[std::make_pair(boundaryEdges[i].first, boundaryEdges[i].second)] == -1);
			assert(edgesToFaces.find(std::make_pair(boundaryEdges[i].second, extremeI))
				== edgesToFaces.end() || edgesToFaces[std::make_pair(boundaryEdges[i].second, extremeI)] == -1);
			assert(edgesToFaces.find(std::make_pair(extremeI, boundaryEdges[i].first))
				== edgesToFaces.end() || edgesToFaces[std::make_pair(extremeI, boundaryEdges[i].first)] == -1);

			if (!(edgesToFaces.find(std::make_pair(boundaryEdges[i].first, boundaryEdges[i].second))
				== edgesToFaces.end() || edgesToFaces[std::make_pair(boundaryEdges[i].first, boundaryEdges[i].second)] == -1)
				|| !(edgesToFaces.find(std::make_pair(boundaryEdges[i].second, extremeI))
				== edgesToFaces.end() || edgesToFaces[std::make_pair(boundaryEdges[i].second, extremeI)] == -1)
			 || !(edgesToFaces.find(std::make_pair(extremeI, boundaryEdges[i].first))
				== edgesToFaces.end() || edgesToFaces[std::make_pair(extremeI, boundaryEdges[i].first)] == -1))
				throw std::exception("Convex hull computation failed!");

			edgesToFaces[std::make_pair(boundaryEdges[i].first, boundaryEdges[i].second)] = (int)p.f.size()-1;
			edgesToFaces[std::make_pair(boundaryEdges[i].second, extremeI)] = (int)p.f.size()-1;
			edgesToFaces[std::make_pair(extremeI, boundaryEdges[i].first)] = (int)p.f.size()-1;
		}
		boundaryEdges.clear();

		// Robustness: flag the new vertex as part of the convex hull.
		if ((int)hullVertices.size() <= extremeI)
			hullVertices.insert(hullVertices.end(), extremeI + 1 - hullVertices.size(), 0);
		//hullVertices[extremeI] = true;
		hullVertices.at(extremeI) = 1;

		// Redistribute all conflicting points to the new faces.
		conflictList.insert(conflictList.end(), p.f.size() - oldNumFaces, std::vector<int>());
		floodFillVisited.insert(floodFillVisited.end(), p.f.size() - oldNumFaces, 0);
		for(std::set<int>::iterator iter = conflictingVertices.begin(); iter != conflictingVertices.end(); ++iter)
			for(size_t j = oldNumFaces; j < p.f.size(); ++j)
			{
				cv ptOnFace = POINT_TO_FLOAT4(p.v[p.f[j].v[0]]);
				cs d = glm::dot(cv(faceNormals[j]), cv(POINT_TO_FLOAT4(p.v[*iter])) - ptOnFace);
//				if (((Plane)facePlanes[j]).IsOnPositiveSide(p.v[*iter]) && (*iter >= (int)hullVertices.size() || !hullVertices[*iter]))
				if (d > inPlaneEpsilon && (*iter >= (int)hullVertices.size() || !hullVertices[*iter]))
				{
					conflictList.at(j).push_back(*iter);
					conflictListVertices[*iter].insert((int)j);
//				conflictList[j].push_back(*iter);
				}
			}

		conflictingVertices.clear();

		// Add new faces that still have conflicting vertices to the work stack for later processing.
		// The algorithm will terminate once all faces are clear of conflicts.
		assert(conflictList.size() == p.f.size());
		for(size_t j = oldNumFaces; j < p.f.size(); ++j)
			if (!conflictList.at(j).empty())
//				if (!conflictList[j].empty())
					workStack.push_back((int)j);
//	p.DumpStructure();
	}

#ifndef NDEBUG
	for(size_t i = 0; i < conflictList.size(); ++i)
		assert(conflictList[i].empty());
#endif

	p.RemoveDegenerateFaces();
	p.RemoveRedundantVertices();

	return p;
}

int IntTriCmp(int a, int b)
{
	return a - b;
}

/** Does a binary search on the array list that is sorted in ascending order.
	@param list [in] A pointer to the array to search.
	@param numItems The number of elements in the array 'list'.
	@param value The element to search for.
	@param cmp The comparison operator to use. The comparison function is of form
		int CmpFunc(const T &a, const T &b), and it tests the mutual order of a and b.
		It should return -1 if a < b, +1 if a > b, and 0 if a == b.
	@return The index where a matching element lies, or -1 if not found. Note that if there are more than
	        one matching element, the first that is found is returned. */
template<typename T, typename CmpFunc>
int ArrayBinarySearch(const T *list, int numItems, const T &value, CmpFunc &cmp)
{
	int left = 0;
	int right = numItems-1;
	int order = cmp(list[left], value);
	if (order > 0) return -1;
	if (order == 0) return left;

	order = cmp(list[right], value);
	if (order < 0) return -1;
	if (order == 0) return right;

	int round = 0; // Counter to alternatingly round up or down.
	do
	{
		int middle = (left + right + round) / 2;
		round = (round+1)&1;
		order = cmp(list[middle], value);
		if (order == 0)
			return middle;
		if (order < 0)
			left = middle;
		else right = middle;
	} while(left < right);
	return -1;
}

void Polyhedron::RemoveDegenerateFaces()
{
	size_t n = 0;
	for(size_t i = 0; i < f.size(); ++i)
	{
		if (f[i].v.size() >= 3)
		{
			if (n != i)
				f[n] = f[i];
			++n;
		}
	}
	f.erase(f.begin()+n, f.end());
}

void Polyhedron::RemoveRedundantVertices()
{
	std::set<int> usedVertices;

	// Gather all used vertices.
	for(size_t i = 0; i < f.size(); ++i)
		for(size_t j = 0; j < f[i].v.size(); ++j)
			usedVertices.insert(f[i].v[j]);

	// Turn the used vertices set into a vector for random access.
	std::vector<int> usedVerticesArray;
	usedVerticesArray.reserve(usedVertices.size());
	for(std::set<int>::iterator iter = usedVertices.begin(); iter != usedVertices.end(); ++iter)
		usedVerticesArray.push_back(*iter);

	// Shift all face indices to point to the new vertex array.
	for(size_t i = 0; i < f.size(); ++i)
		for(size_t j = 0; j < f[i].v.size(); ++j)
		{
			int oldIndex = f[i].v[j];
			int newIndex = ArrayBinarySearch(&usedVerticesArray[0], (int)usedVerticesArray.size(), oldIndex, IntTriCmp);
			assert(newIndex != -1);
			f[i].v[j] = newIndex;
		}

	// Delete all unused vertices from the vertex array.
	for(size_t i = 0; i < usedVerticesArray.size(); ++i)
		v[i] = v[usedVerticesArray[i]];
	v.resize(usedVerticesArray.size());
}
