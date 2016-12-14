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

/** @file OBB.cpp
	@author Jukka Jylänki
	@brief Implementation for the Oriented Bounding Box (OBB) geometry object. */
#include "OBB.h"
#include "float3x3.h"
#include <random>
#include "MathLog.h"
#include "../Clock.h"

#define FLOAT_INF INFINITY
#define DIR_VEC vec
#define assume ((void)0)
#define MARK_UNUSED ((void)0)
#define FORCE_INLINE __inline

OBB::OBB(const vec &pos, const vec &r, const vec &axis0, const vec &axis1, const vec &axis2)
:pos(pos), r(r)
{
	axis[0] = axis0;
	axis[1] = axis1;
	axis[2] = axis2;
}

bool OBB::IsDegenerate() const
{
	return !(r.x > 0.f && r.y > 0.f && r.z > 0.f);
}

vec OBB::CenterPoint() const
{
	return pos;
}

vec OBB::CornerPoint(int cornerIndex) const
{	
	assume(0 <= cornerIndex && cornerIndex <= 7);
	switch(cornerIndex)
	{
		default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
		case 0: return pos - r.x * axis[0] - r.y * axis[1] - r.z * axis[2];
		case 1: return pos - r.x * axis[0] - r.y * axis[1] + r.z * axis[2];
		case 2: return pos - r.x * axis[0] + r.y * axis[1] - r.z * axis[2];
		case 3: return pos - r.x * axis[0] + r.y * axis[1] + r.z * axis[2];
		case 4: return pos + r.x * axis[0] - r.y * axis[1] - r.z * axis[2];
		case 5: return pos + r.x * axis[0] - r.y * axis[1] + r.z * axis[2];
		case 6: return pos + r.x * axis[0] + r.y * axis[1] - r.z * axis[2];
		case 7: return pos + r.x * axis[0] + r.y * axis[1] + r.z * axis[2];
	}
}

vec OBB::ExtremePoint(const vec &direction) const
{
	vec pt = pos;
	pt += axis[0] * (Dot(direction, axis[0]) >= 0.f ? r.x : -r.x);
	pt += axis[1] * (Dot(direction, axis[1]) >= 0.f ? r.y : -r.y);
	pt += axis[2] * (Dot(direction, axis[2]) >= 0.f ? r.z : -r.z);
	return pt;
}

vec OBB::ExtremePoint(const vec &direction, float &projectionDistance) const
{
	vec extremePoint = ExtremePoint(direction);
	projectionDistance = extremePoint.Dot(direction);
	return extremePoint;
}

void OBB::ProjectToAxis(const vec &direction, float &outMin, float &outMax) const
{
	float x = glm::Abs(glm::Dot(direction, axis[0]) * r.x);
	float y = glm::Abs(glm::Dot(direction, axis[1]) * r.y);
	float z = glm::Abs(glm::Dot(direction, axis[2]) * r.z);
	float pt = glm::Dot(direction, pos);
	outMin = pt - x - y - z;
	outMax = pt + x + y + z;
}

int OBB::UniqueFaceNormals(vec *out) const
{
	out[0] = axis[0];
	out[1] = axis[1];
	out[2] = axis[2];
	return 3;
}

int OBB::UniqueEdgeDirections(vec *out) const
{
	out[0] = axis[0];
	out[1] = axis[1];
	out[2] = axis[2];
	return 3;
}

vec OBB::PointOnEdge(int edgeIndex, float u) const
{
	assume(0 <= edgeIndex && edgeIndex <= 11);
	assume(0 <= u && u <= 1.f);

	edgeIndex = glm::clamp(edgeIndex, 0, 11);
	vec d = axis[edgeIndex/4] * (2.f * u - 1.f) * r[edgeIndex/4];
	switch(edgeIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return pos - r.y * axis[1] - r.z * axis[2] + d;
	case 1: return pos - r.y * axis[1] + r.z * axis[2] + d;
	case 2: return pos + r.y * axis[1] - r.z * axis[2] + d;
	case 3: return pos + r.y * axis[1] + r.z * axis[2] + d;

	case 4: return pos - r.x * axis[0] - r.z * axis[2] + d;
	case 5: return pos - r.x * axis[0] + r.z * axis[2] + d;
	case 6: return pos + r.x * axis[0] - r.z * axis[2] + d;
	case 7: return pos + r.x * axis[0] + r.z * axis[2] + d;

	case 8: return pos - r.x * axis[0] - r.y * axis[1] + d;
	case 9: return pos - r.x * axis[0] + r.y * axis[1] + d;
	case 10: return pos + r.x * axis[0] - r.y * axis[1] + d;
	case 11: return pos + r.x * axis[0] + r.y * axis[1] + d;
	}
}

vec OBB::FaceCenterPoint(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);

	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return pos - r.x * axis[0];
	case 1: return pos + r.x * axis[0];
	case 2: return pos - r.y * axis[1];
	case 3: return pos + r.y * axis[1];
	case 4: return pos - r.z * axis[2];
	case 5: return pos + r.z * axis[2];
	}
}

vec OBB::FacePoint(int faceIndex, float u, float v) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	assume(0 <= u && u <= 1.f);
	assume(0 <= v && v <= 1.f);

	int uIdx = faceIndex/2;
	int vIdx = (faceIndex/2 + 1) % 3;
	vec U = axis[uIdx] * (2.f * u - 1.f) * r[uIdx];
	vec V = axis[vIdx] * (2.f * v - 1.f) * r[vIdx];
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return pos - r.z * axis[2] + U + V;
	case 1: return pos + r.z * axis[2] + U + V;
	case 2: return pos - r.x * axis[0] + U + V;
	case 3: return pos + r.x * axis[0] + U + V;
	case 4: return pos - r.y * axis[1] + U + V;
	case 5: return pos + r.y * axis[1] + U + V;
	}
}

void OBB::GetCornerPoints(vec *outPointArray) const
{
	assume(outPointArray);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!outPointArray)
		return;
#endif
	for(int i = 0; i < 8; ++i)
		outPointArray[i] = CornerPoint(i);
}

/// See Christer Ericson's book Real-Time Collision Detection, page 83.
void OBB::ExtremePointsAlongDirection(const vec &dir, const vec *pointArray, int numPoints, int &idxSmallest, int &idxLargest, float &smallestD, float &largestD)
{
	assume(pointArray || numPoints == 0);

	idxSmallest = idxLargest = 0;

#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!pointArray)
		return;
#endif

	smallestD = FLOAT_INF;
	largestD = -FLOAT_INF;
	for(int i = 0; i < numPoints; ++i)
	{
		float d =glm::Dot(pointArray[i], dir);
		if (d < smallestD)
		{
			smallestD = d;
			idxSmallest = i;
		}
		if (d > largestD)
		{
			largestD = d;
			idxLargest = i;
		}
	}
}

static bool AreEdgesCompatibleForOBB(const vec &f1a, const vec &f1b, const vec &f2a, const vec &f2b)
{
	const vec f1a_f1b = f1a-f1b;
	const vec f2a_f2b = f2a-f2b;
	float a = f1b.Dot(f2b);
	float b = (f1a_f1b).Dot(f2b);
	float c = (f2a_f2b).Dot(f1b);
	float d = (f1a_f1b).Dot(f2a_f2b);

	/*
		n1 = f1a*t + f1b*(1-t) = f1b + (f1a-f1b)*t
		n2 = f2a*u + f2b*(1-u) = f2b + (f2a-f2b)*u

		n1.n2 = 0
			f1b.f2b + t*(f1a-f1b).f2b + u*(f2a-f2b).f1b + t*u*(f1a-f1b).(f2a-f2b) = 0
			a + t*b + u*c + t*u*d = 0

		Does this equation have a solution within t & u \in [0,1]?

		// The function f(t,u) = a + t*b + u*c + t*u*d is continuous and bilinear
		// with respect to t and u, so test the four corners to get the minimum
		// and maximum of the function. If minimum <= 0 and
		// maximum >= 0, we know it must have a zero inside t,u \in [0,1].

		t=0: f(t,u)=a+uc   => min: a, max: a+c
		u=0: f(t,u)=a+tb   => min: a, max: a+b
		t=1: f(t,u)=a+uc + b+ud  => min: a+b, max: a+b+c+d
		u=1: f(t,u)=a+tb + c+td  => min: a+c, max: a+b+c+d
	*/
	float ab = a+b;
	float ac = a+c;
	float abcd = ab+c+d;
	float minVal = glm::min(a, ab, ac, abcd);
	float maxVal = glm::max(a, ab, ac, abcd);
	return minVal <= 0.f && maxVal >= 0.f;
}

// Enable this to add extra runtime checks to sanity test that the generated OBB is actually valid.
//#define OBB_ASSERT_VALIDITY

// Enable this to add internal debug prints for tracking internal behavior.
//#define OBB_DEBUG_PRINT

// Enable this to print out detailed profiling info.
#define ENABLE_TIMING

#ifdef ENABLE_TIMING
#define TIMING_TICK(...) __VA_ARGS__
#define TIMING LOGI
#else
#define TIMING(...) ((void)0)
#define TIMING_TICK(...) ((void)0)
#endif

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


OBB OBB::OptimalEnclosingOBB(const vec *pointArray, int numPoints)
{
	// Precomputation: Generate the convex hull of the input point set. This is because
	// we need vertex-edge-face connectivity information about the convex hull shape, and
	// this also allows discarding all points in the interior of the input hull, which
	// are irrelevant.
	Polyhedron convexHull = Polyhedron::ConvexHull(pointArray, numPoints);
	return OptimalEnclosingOBB(convexHull);
}

bool IsVertexAntipodalToEdge(const Polyhedron &convexHull, int vi, const std::vector<int> &neighbors, const vec &f1a, const vec &f1b)
{
	float tMin = 0.f;
	float tMax = 1.f;

	vec v = convexHull.v[vi];
	vec f1b_f1a = f1b-f1a;
	for(size_t i = 0; i < neighbors.size(); ++i)
	{
		/* Is an edge and a vertex compatible to be antipodal?

			n1 = f1b + (f1a-f1b)*t
			e { v-vn }

			n1.e <= 0
			(f1b + (f1a-f1b)*t).e <= 0
			t*(f1a-f1b).e <= -f1b.e 
			if (f1a-f1b).e > 0:
				t <= -f1b.e / (f1a-f1b).e && t \in [0,1]
				-f1b.e / (f1a-f1b).e >= 0
				f1b.e <= 0
			if (f1a-f1b).e < 0:
				t >= -f1b.e / (f1a-f1b).e && t \in [0,1]
				-f1b.e / (f1a-f1b).e <= 1
				-f1b.e >= (f1a-f1b).e
				f1b.e + (f1a-f1b).e <= 0 
			if (f1a-f1b).e == 0:
				0 <= -f1b.e
		*/
		vec e = vec(convexHull.v[neighbors[i]]) - v;
		float s = f1b_f1a.Dot(e);
		float n = f1b.Dot(e);
		const float epsilon = 1e-4f;
		if (s > epsilon)
			tMax = glm::min(tMax, n / s);
		else if (s < -epsilon)
			tMin = glm::max(tMin, n / s);
		else if (n < -epsilon)
			return false;

		// The interval of possible solutions for t is now degenerate?
		if (tMax - tMin < -5e-2f) // -1e-3f has been seen to be too strict here.
			return false;
	}
	return true;
}

OBB OBB::OptimalEnclosingOBB(const Polyhedron &convexHull)
{
	/* Outline of the algorithm:
	  0. Compute the convex hull of the point set (given as input to this function) O(VlogV)
	  1. Compute vertex adjacency data, i.e. given a vertex, return a list of its neighboring vertices. O(V)
	  2. Precompute face normal direction vectors, since these are needed often. (does not affect big-O complexity, just a micro-opt) O(F)
	  3. Compute edge adjacency data, i.e. given an edge, return the two indices of its neighboring faces. O(V)
	  4. Precompute antipodal vertices for each edge. O(A*ElogV), where A is the size of antipodal vertices per edge. A ~ O(1) on average.
	  5. Precompute all sidepodal edges for each edge. O(E*S), where S is the size of sidepodal edges per edge. S ~ O(sqrtE) on average.
	     - Sort the sidepodal edges to a linear order so that it's possible to do fast set intersection computations on them. O(E*S*logS), or O(E*sqrtE*logE).
	  6. Test all configurations where all three edges are on adjacent faces. O(E*S^2) = O(E^2) or if smart with graph search, O(ES) = O(E*sqrtE)?
	  7. Test all configurations where two edges are on opposing faces, and the third one is on a face adjacent to the two. O(E*sqrtE*logV)?
	  8. Test all configurations where two edges are on the same face (OBB aligns with a face of the convex hull). O(F*sqrtE*logV).
	  9. Return the best found OBB.
	*/

	OBB minOBB;
	float minVolume = FLOAT_INF;

	TIMING("#### OBB TIMING BEGIN ####");
	TIMING_TICK(tick_t t1 = Clock::Tick());
	// Precomputation: For each vertex in the convex hull, compute their neighboring vertices.
	std::vector<std::vector<int> > adjacencyData = convexHull.GenerateVertexAdjacencyData(); // O(V)
	TIMING_TICK(tick_t t2 = Clock::Tick());
	TIMING("Adjacencygeneration: %f msecs", Clock::TimespanToMillisecondsF(t1, t2));

	// Precomputation: Compute normalized face direction vectors for each face of the hull.
	//std::vector<vec_storage> faceNormals;
	VecArray faceNormals;
	faceNormals.reserve(convexHull.NumFaces());
	for(int i = 0; i < convexHull.NumFaces(); ++i) // O(F)
	{
		if (convexHull.f[i].v.size() < 3)
		{
			throw std::exception("Input convex hull contains a degenerate face %d with only %d vertices! Cannot process this!");
		}
		vec normal = convexHull.FaceNormal(i);
		faceNormals.push_back(DIR_VEC((float)normal.x, (float)normal.y, (float)normal.z));
	}

	TIMING_TICK(tick_t t23 = Clock::Tick());
	TIMING("Facenormalsgen: %f msecs", Clock::TimespanToMillisecondsF(t2, t23));

#pragma region AddjoiningFaces
	// For each edge i, specifies the two vertices v0 and v1 through which that edge goes.
	// This array does not have duplicates, i.e. there only exists one edge index for (v0->v1), and no
	// edge for (v1->v0).
	std::vector<std::pair<int, int> > edges;
	edges.reserve(convexHull.v.size() * 2);
	// For each edge i, specifies the two face indices f0 and f1 that share that edge.
	std::vector<std::pair<int, int> > facesForEdge;
	facesForEdge.reserve(convexHull.v.size()*2);
	// For each vertex pair (v0, v1) through which there is an edge, specifies the index i of the edge that passes through them.
	// This map contains duplicates, so both (v0, v1) and (v1, v0) map to the same edge index.
	// Currently use a O(V^2) array for this data structure for performance.
	// TODO: Add compile option for using unordered_map for O(V) storage, but that's somewhat slower.
	//	std::unordered_map<std::pair<int, int>, int, hash_edge> vertexPairsToEdges;
	unsigned int *vertexPairsToEdges = new unsigned int[convexHull.v.size()*convexHull.v.size()];
	memset(vertexPairsToEdges, 0xFF, sizeof(unsigned int)*convexHull.v.size()*convexHull.v.size());
#define EMPTY_EDGE 0xFFFFFFFFU

	// Precomputation: for each edge through vertices (i,j), we need to know
	// the face indices for the two adjoining faces that share the edge.
	// This is O(V).
	for (size_t i = 0; i < convexHull.f.size(); ++i)
	{
		const Polyhedron::Face &f = convexHull.f[i];
		int v0 = f.v.back();
		for(size_t j = 0; j < f.v.size(); ++j)
		{
			int v1 = f.v[j];
			std::pair<int, int> e = std::make_pair(v0, v1);
			//std::unordered_map<std::pair<int, int>, int, hash_edge>::const_iterator iter = vertexPairsToEdges.find(e);
			//if (iter == vertexPairsToEdges.end())
			if (vertexPairsToEdges[v0*convexHull.v.size()+v1] == EMPTY_EDGE)
			{
				//vertexPairsToEdges[e] = (int)edges.size();
				vertexPairsToEdges[v0*convexHull.v.size()+v1] = (unsigned int)edges.size();
				vertexPairsToEdges[v1*convexHull.v.size()+v0] = (unsigned int)edges.size();
//				vertexPairsToEdges[std::make_pair(v1, v0)] = (int)edges.size(); // Mark that we know we have seen v0->v1 already.
				edges.push_back(e);
				facesForEdge.push_back(std::make_pair((int)i, -1)); // The -1 will be filled once we see the edge v1->v0.
			}
			else
				facesForEdge[vertexPairsToEdges[v0*(int)convexHull.v.size()+v1]/*iter->second*/].second = (int)i;
			v0 = v1;
		}
	}
	TIMING_TICK(tick_t t3 = Clock::Tick());
	TIMING("Adjoiningfaces: %f msecs", Clock::TimespanToMillisecondsF(t23, t3));

#pragma endregion AddjoiningFaces

	// Throughout the algorithm, internal edges can all be discarded, so provide a helper macro to test for that.
#define IS_INTERNAL_EDGE(i) (reinterpret_cast<vec*>(&faceNormals[facesForEdge[i].first])->Dot(faceNormals[facesForEdge[i].second]) > 1.f - 1e-4f)

	TIMING_TICK(
		int numInternalEdges = 0;
		for(size_t i = 0; i < edges.size(); ++i)
			if (IS_INTERNAL_EDGE(i))
				++numInternalEdges;
	);
	TIMING("%d/%d (%.2f%%) edges are internal and will be ignored.", numInternalEdges, (int)edges.size(), numInternalEdges * 100.0 / edges.size());

	// Throughout the whole algorithm, this array stores an auxiliary structure for performing graph searches
	// on the vertices of the convex hull. Conceptually each index of the array stores a boolean whether we
	// have visited that vertex or not during the current search. However storing such booleans is slow, since
	// we would have to perform a linear-time scan through this array before next search to reset each boolean
	// to unvisited false state. Instead, store a number, called a "color" for each vertex to specify whether
	// that vertex has been visited, and manage a global color counter floodFillVisitColor that represents the
	// visited vertices. At any given time, the vertices that have already been visited have the value
	// floodFillVisited[i] == floodFillVisitColor in them. This gives a win that we can perform constant-time
	// clears of the floodFillVisited array, by simply incrementing the "color" counter to clear the array.
	std::vector<unsigned int> floodFillVisited(convexHull.v.size());
	unsigned int floodFillVisitColor = 1;

	// As a syntactic aid, use the helpers MARK_VISITED(v), HAVE_VISITED_VERTEX(v) and CLEAR_GRAPH_SEARCH to
	// remind of the conceptual meaning of these values.
#define MARK_VERTEX_VISITED(v) (floodFillVisited[(v)] = floodFillVisitColor)
#define HAVE_VISITED_VERTEX(v) (floodFillVisited[(v)] == floodFillVisitColor) // HAVE_VISITED_VERTEX(v) implies HAVE_QUEUED_VERTEX(v)
#define CLEAR_GRAPH_SEARCH() (++floodFillVisitColor)

#pragma region SpatialFaceOrder 

	TIMING_TICK(
		tick_t tz = Clock::Tick();
		unsigned long long numSpatialStrips = 0);

	// The currently best variant for establishing a spatially coherent traversal order.
	std::vector<int> spatialFaceOrder;

	std::linear_congruential_engine<unsigned int, 69621, 0, 0x7FFFFFFF> eng;
	{ // Explicit scope for variables that are not needed after this.
		TIMING_TICK(int prevEdgeEnd = -1);
		std::vector<std::pair<int, int> > traverseStackEdges;
		std::vector<unsigned int> visitedEdges(edges.size());
		std::vector<unsigned int> visitedFaces(convexHull.f.size());
		traverseStackEdges.push_back(std::make_pair(0, adjacencyData[0].front()));
		while(!traverseStackEdges.empty())
		{
			std::pair<int, int> e = traverseStackEdges.back();
			traverseStackEdges.pop_back();
			int thisEdge = vertexPairsToEdges[e.first*convexHull.v.size()+e.second];
			if (visitedEdges[thisEdge])
				continue;
			visitedEdges[thisEdge] = 1;
			if (!visitedFaces[facesForEdge[thisEdge].first])
			{
				visitedFaces[facesForEdge[thisEdge].first] = 1;
				spatialFaceOrder.push_back(facesForEdge[thisEdge].first);
			}
			if (!visitedFaces[facesForEdge[thisEdge].second])
			{
				visitedFaces[facesForEdge[thisEdge].second] = 1;
				spatialFaceOrder.push_back(facesForEdge[thisEdge].second);
			}
			TIMING_TICK(
				if (prevEdgeEnd != e.first)
					++numSpatialStrips;
				prevEdgeEnd = e.second;
			);

			int v0 = e.second;
			size_t sizeBefore = traverseStackEdges.size();
			for(size_t i = 0; i < adjacencyData[v0].size(); ++i)
			{
				int v1 = adjacencyData[v0][i];
				int e1 = vertexPairsToEdges[v0*convexHull.v.size()+v1];
				if (visitedEdges[e1])
					continue;
				traverseStackEdges.push_back(std::make_pair(v0, v1));
			}
			// Take a random adjacent edge.
			int nNewEdges = (int)(traverseStackEdges.size() - sizeBefore);
			if (nNewEdges > 0)
			{
				std::uniform_int_distribution<> rngRange(0, nNewEdges - 1); // define the range
				auto r = rngRange(eng);
				std::swap(traverseStackEdges.back(), traverseStackEdges[sizeBefore+r]);
			}
		}
	}


	TIMING_TICK(tick_t tx = Clock::Tick());
	TIMING("SpatialOrder: %f msecs, search over %d edges is in %llu strips, approx. %f edges/strip",
		Clock::TimespanToMillisecondsF(tz, tx), (int)edges.size(), numSpatialStrips, (double)edges.size()/numSpatialStrips);
#pragma endregion SpatialFaceOrder

#pragma region Antipodal
	// Stores for each edge index i the complete list of antipodal vertices for that edge.
	std::vector<std::vector<int> > antipodalPointsForEdge(edges.size());

	// A naive O(E*V) algorithm for computing all antipodal points for each edge.
	for (size_t i = 0; i < edges.size(); ++i) // O(E)
	{
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];
		for(size_t j = 0; j < convexHull.v.size(); ++j) // O(V)
			if (IsVertexAntipodalToEdge(convexHull, j, adjacencyData[j], f1a, f1b))
				antipodalPointsForEdge[i].push_back(j);
	}

	TIMING_TICK(
		tick_t t4 = Clock::Tick();
		size_t numTotalAntipodals = 0;
		for (size_t i = 0; i < antipodalPointsForEdge.size(); ++i)
			numTotalAntipodals += antipodalPointsForEdge[i].size();
	);
	TIMING("Antipodalpoints: %f msecs (avg edge has %.3f antipodal points)", Clock::TimespanToMillisecondsF(tx, t4), (float)numTotalAntipodals/edges.size());
#pragma endregion Antipodal

#pragma region Sidepodal
	// Stores for each edge i the list of all sidepodal edge indices j that it can form an OBB with.
	std::vector<std::vector<int> > compatibleEdges(edges.size());

	// Precomputation: Compute all potential companion edges for each edge.
	// This is O(E^2)
	// Important! And edge can be its own companion edge! So have each edge test itself during iteration.
	for(size_t i = 0; i < edges.size(); ++i) // O(E)
	{
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];
		for(size_t j = i; j < edges.size(); ++j) // O(E)
			if (AreEdgesCompatibleForOBB(f1a, f1b, faceNormals[facesForEdge[j].first], faceNormals[facesForEdge[j].second]))
			{
				compatibleEdges[i].push_back(j);

				if (i != j)
					compatibleEdges[j].push_back(i);
			}
	}

	TIMING_TICK(
		tick_t t5 = Clock::Tick();
		size_t numTotalEdges = 0;
		for(size_t i = 0; i < compatibleEdges.size(); ++i)
			numTotalEdges += compatibleEdges[i].size();
	);
	TIMING("Companionedges: %f msecs (%d edges have on average %d companion edges each)", Clock::TimespanToMillisecondsF(t4, t5), (int)compatibleEdges.size(), (int)(numTotalEdges / compatibleEdges.size()));
	TIMING_TICK(t5 = Clock::Tick());
#pragma endregion Sidepodal

#pragma region Faceconfigs
	// Take advantage of spatial locality: start the search for the extreme vertex from the extreme vertex
	// that was found during the previous iteration for the previous edge. This speeds up the search since
	// edge directions have some amount of spatial locality and the next extreme vertex is often close
	// to the previous one. Track two hint variables since we are performing extreme vertex searches to
	// two opposing directions at the same time.
	int extremeVertexSearchHint1 = 0;
	int extremeVertexSearchHint2 = 0;
	int extremeVertexSearchHint3 = 0;

	TIMING_TICK(
		tick_t t72 = Clock::Tick();
		int numTwoSameFacesConfigs = 0;
		int numVertexNeighborSearches = 0;
		int numVertexNeighborSearchImprovements = 0;
		);

	// Main algorithm body for computing all search directions where the OBB touches two edges on the same face.
	// This is O(F*sqrtE*logV)?
	for(size_t ii = 0; ii < spatialFaceOrder.size(); ++ii) // O(F)
	//for(size_t i = 0; i < faceNormals.size(); ++i) // O(F)
	{
		size_t i = spatialFaceOrder[ii];
		vec n1 = faceNormals[i];

		// Find two edges on the face. Since we have flexibility to choose from multiple edges of the same face,
		// choose two that are possibly most opposing to each other, in the hope that their sets of sidepodal
		// edges are most mutually exclusive as possible, speeding up the search below.
		int e1 = -1;
		int v0 = convexHull.f[i].v.back();
		for(size_t j = 0; j < convexHull.f[i].v.size(); ++j)
		{
			int v1 = convexHull.f[i].v[j];
			int e = vertexPairsToEdges[v0*convexHull.v.size()+v1];
			if (!IS_INTERNAL_EDGE(e))
			{
				e1 = e;
				break;
			}
			v0 = v1;
		}
		if (e1 == -1)
			continue; // All edges of this face were degenerate internal edges! Just skip processing the whole face.

		const std::vector<int> &antipodals = antipodalPointsForEdge[e1];
		const std::vector<int> &compatibleEdgesI = compatibleEdges[e1];

		float maxN1 = n1.Dot(convexHull.v[edges[e1].first]);
		float minN1;

		minN1 = FLOAT_INF;
		for(size_t j = 0; j < antipodals.size(); ++j)
			minN1 = glm::min(minN1, n1.Dot(convexHull.v[antipodals[j]]));

		for(size_t j = 0; j < compatibleEdgesI.size(); ++j)
		{
			int edge3 = compatibleEdgesI[j];
			// Test all mutual compatible edges.
			vec f3a = faceNormals[facesForEdge[edge3].first];
			vec f3b = faceNormals[facesForEdge[edge3].second];

			// Is edge3 compatible with direction n?
			// n3 = f3b + (f3a-f3b)*v
			// n1.n3 = 0
			// n1.(f3b + (f3a-f3b)*v) = 0
			// n1.f3b + n1.((f3a-f3b)*v) = 0
			// n1.f3b = (n1.(f3b-f3a))*v
			// If n1.(f3b-f3a) != 0:
			//    v = n1.f3b / n1.(f3b-f3a)
			// If n1.(f3b-f3a) == 0:
			//    n1.f3b must be zero as well, then arbitrary v is ok.
			float num = n1.Dot(f3b);
			float denom = n1.Dot(f3b-f3a);
			float v;
			if (!glm::EqualAbs(denom, 0.f))
				v = num / denom;
			else
				v = glm::EqualAbs(num, 0.f) ? 0.f : -1.f;

			const float epsilon = 1e-4f;
			if (v >= 0.f - epsilon && v <= 1.f + epsilon)
			{
				vec n3 = vec(f3b + (f3a - f3b) * v).Normalized();
				vec n2 = n3.Cross(n1).Normalized();

				float minN2, maxN2;
				CLEAR_GRAPH_SEARCH();
				extremeVertexSearchHint1 = convexHull.ExtremeVertexConvex(adjacencyData, n2, floodFillVisited, floodFillVisitColor, maxN2, extremeVertexSearchHint1); // O(logV)?
				TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
				TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
				CLEAR_GRAPH_SEARCH();
				extremeVertexSearchHint2 = convexHull.ExtremeVertexConvex(adjacencyData, -n2, floodFillVisited, floodFillVisitColor, minN2, extremeVertexSearchHint2); // O(logV)?
				TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
				TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
				minN2 = -minN2;
				float maxN3 = n3.Dot(convexHull.v[edges[edge3].first]);

				const std::vector<int> &antipodalsEdge3 = antipodalPointsForEdge[edge3];
				float minN3 = FLOAT_INF;
				if (antipodalsEdge3.size() < 20) // If there are very few antipodal vertices, do a very tight loop and just iterate over each.
				{
					for(size_t a = 0; a < antipodalsEdge3.size(); ++a)
						minN3 = glm::min(minN3, n3.Dot(convexHull.v[antipodalsEdge3[a]]));
				}
				else
				{
					// Otherwise perform a spatial locality exploiting graph search.
					CLEAR_GRAPH_SEARCH();
					extremeVertexSearchHint3 = convexHull.ExtremeVertexConvex(adjacencyData, -n3, floodFillVisited, floodFillVisitColor, minN3, extremeVertexSearchHint3); // O(logV)?
					TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
					TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
					minN3 = -minN3;
				}

				float volume = (maxN1 - minN1) * (maxN2 - minN2) * (maxN3 - minN3);
				TIMING_TICK(++numTwoSameFacesConfigs);
				if (volume < minVolume)
				{
					minOBB.pos = ((minN1 + maxN1) * n1 + (minN2 + maxN2) * n2 + (minN3 + maxN3) * n3) * 0.5f;
					minOBB.axis[0] = n1;
					minOBB.axis[1] = n2;
					minOBB.axis[2] = n3;
					minOBB.r[0] = (maxN1 - minN1) * 0.5f;
					minOBB.r[1] = (maxN2 - minN2) * 0.5f;
					minOBB.r[2] = (maxN3 - minN3) * 0.5f;
					assert(volume > 0.f);
					minVolume = volume;
				}
			}
		}
	}

	TIMING_TICK(tick_t t8 = Clock::Tick());
	TIMING("Faceconfigs: %f msecs (%d configs, %d faces, %f configs/face)", Clock::TimespanToMillisecondsF(t72, t8), numTwoSameFacesConfigs, (int)spatialFaceOrder.size(), (double)numTwoSameFacesConfigs/spatialFaceOrder.size());
	TIMING("%f vertex neighbor searches per edge, %f improvements", (double)numVertexNeighborSearches/numTwoSameFacesConfigs, (double)numVertexNeighborSearchImprovements/numTwoSameFacesConfigs);
#pragma endregion Faceconfigs
	TIMING("#### OBB TIMING END ####");

	// The search for edge triplets does not follow cross-product orientation, so
	// fix that up at the very last step, if necessary.
	if (minOBB.axis[0].Cross(minOBB.axis[1]).Dot(minOBB.axis[2]) < 0.f)
		minOBB.axis[2] = -minOBB.axis[2];
#ifdef MATH_VEC_IS_FLOAT4
	minOBB.r.w = 0.f;
	minOBB.pos.w = 1.f;
#endif
	delete[] vertexPairsToEdges;
	return minOBB;
}

vec OBB::Size() const
{
	return r * 2.f;
}

vec OBB::HalfSize() const
{
	return r;
}

vec OBB::Diagonal() const
{
	return 2.f * HalfDiagonal();
}

vec OBB::HalfDiagonal() const
{
	return axis[0] * r[0] + axis[1] * r[1] + axis[2] * r[2];
}

float OBB::Volume() const
{
	vec size = Size();
	return size.x*size.y*size.z;
}

float OBB::SurfaceArea() const
{
	const vec size = Size();
	return 2.f * (size.x*size.y + size.x*size.z + size.y*size.z);
}
