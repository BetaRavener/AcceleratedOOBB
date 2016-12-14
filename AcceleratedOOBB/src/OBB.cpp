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

//float SmallestOBBVolumeJiggle(const vec &edge_, const Polyhedron &convexHull, std::vector<float2> &pts,
//	vec &outEdgeA, vec &outEdgeB)
//{
//	vec edge = edge_;
//	int numTimesNotImproved = 0;
//	float bestVolume = FLOAT_INF;
//	float2 c10, c20;
//	vec u, v;
//	vec prevSecondChoice = vec::nan;
//	int numJiggles = 2;
//	while(numTimesNotImproved < 2)
//	{
//		int e1, e2;
//		OBB::ExtremePointsAlongDirection(edge, (const vec*)&convexHull.v[0], (int)convexHull.v.size(), e1, e2);
//		float edgeLength = glm::Abs(Dot((vec)convexHull.v[e1] - convexHull.v[e2], edge));
//
//		edge.PerpendicularBasis(u, v);
//
//		for(size_t k = 0; k < convexHull.v.size(); ++k)
//			pts[k] = float2(u.Dot(convexHull.v[k]), v.Dot(convexHull.v[k]));
//
//		float2 rectCenter;
//		float2 uDir;
//		float2 vDir;
//		float minU, maxU, minV, maxV;
//		float rectArea = float2::MinAreaRectInPlace(&pts[0], (int)pts.size(), rectCenter, uDir, vDir, minU, maxU, minV, maxV);
//
//		c10 = (maxV - minV) * vDir;
//		c20 = (maxU - minU) * uDir;
//
//		float volume = rectArea*edgeLength;
//		if (volume + 1e-5f < bestVolume)
//		{
//			bestVolume = volume;
//			edge = (c10.x*u + c10.y*v);
//			float len = edge.Normalize();
//			if (len <= 0.f)
//				edge = u;
//			numTimesNotImproved = 0;
//			prevSecondChoice = (c20.x*u + c20.y*v);
//			len = prevSecondChoice.Normalize();
//			if (len <= 0.f)
//				prevSecondChoice = u;
//			outEdgeA = edge;
//			outEdgeB = prevSecondChoice;
//
//// Enable for a dirty hack to experiment with performance.
////#define NO_JIGGLES
//
//#ifdef NO_JIGGLES
//			break;
//#endif
//		}
//		else
//		{
//			++numTimesNotImproved;
//			edge = prevSecondChoice;
//		}
//
//		if (--numJiggles <= 0)
//			break;
//	}
//	return bestVolume;
//}

// Moves the floating point sign bit from src to dst.
#ifdef MATH_SSE
#define MoveSign(dst, src) \
	dst = s4f_x(xor_ps(setx_ps(dst), and_ps(setx_ps(src), simd4fSignBit))); \
	src = s4f_x(glm::Abs_ps(setx_ps(src)));
#else
#define MoveSign(dst, src) if (src < 0.f) { dst = -dst; src = -src; }
#endif

int ComputeBasis(const vec &f1a, const vec &f1b,
	const vec &f2a, const vec &f2b,
	const vec &f3a, const vec &f3b,
	vec *n1,
	vec *n2,
	vec *n3)
{
	const float eps = 1e-4f;
	const float angleEps = 1e-3f;

	{
		vec a = f1b;
		vec b = f1a-f1b;
		vec c = f2b;
		vec d = f2a-f2b;
		vec e = f3b;
		vec f = f3a-f3b;

		float g = a.Dot(c)*d.Dot(e) - a.Dot(d)*c.Dot(e);
		float h = a.Dot(c)*d.Dot(f) - a.Dot(d)*c.Dot(f);
		float i = b.Dot(c)*d.Dot(e) - b.Dot(d)*c.Dot(e);
		float j = b.Dot(c)*d.Dot(f) - b.Dot(d)*c.Dot(f);

		float k = g*b.Dot(e) - a.Dot(e)*i;
		float l = h*b.Dot(e) + g*b.Dot(f) - a.Dot(f)*i - a.Dot(e)*j;
		float m = h*b.Dot(f) - a.Dot(f)*j;

		float s = l*l - 4*m*k;

		if (glm::Abs(m) < 1e-5f || glm::Abs(s) < 1e-5f)
		{
			// The equation is linear instead.

			float v = -k / l;
			float t = -(g + h*v) / (i + j*v);
			float u = -(c.Dot(e) + c.Dot(f)*v) / (d.Dot(e) + d.Dot(f)*v);
			int nSolutions = 0;
			// If we happened to divide by zero above, the following checks handle them.
			if (v >= -eps && t >= -eps && u >= -eps && v <= 1.f + eps && t <= 1.f + eps && u <= 1.f + eps)
			{
				n1[0] = vec(a + b*t).Normalized();
				n2[0] = vec(c + d*u).Normalized();
				n3[0] = vec(e + f*v).Normalized();
				if (glm::Abs(n1[0].Dot(n2[0])) < angleEps
					&& glm::Abs(n1[0].Dot(n3[0])) < angleEps
					&& glm::Abs(n2[0].Dot(n3[0])) < angleEps)
					return 1;
				else
					return 0;
			}
			return nSolutions;
		}

		if (s < 0.f)
			return 0; // Discriminant negative, no solutions for v.

		float sgnL = l < 0 ? -1.f : 1.f;
		float V1 = -(l + sgnL * glm::sqrt(s))/ (2.f*m);
		float V2 = k / (m*V1);

		float T1 = -(g + h*V1) / (i + j*V1);
		float T2 = -(g + h*V2) / (i + j*V2);

		float U1 = -(c.Dot(e) + c.Dot(f)*V1) / (d.Dot(e) + d.Dot(f)*V1);
		float U2 = -(c.Dot(e) + c.Dot(f)*V2) / (d.Dot(e) + d.Dot(f)*V2);

		int nSolutions = 0;
		if (V1 >= -eps && T1 >= -eps && U1 >= -eps && V1 <= 1.f + eps && T1 <= 1.f + eps && U1 <= 1.f + eps)
		{
			n1[nSolutions] = vec(a + b*T1).Normalized();
			n2[nSolutions] = vec(c + d*U1).Normalized();
			n3[nSolutions] = vec(e + f*V1).Normalized();

			if (glm::Abs(n1[nSolutions].Dot(n2[nSolutions])) < angleEps
				&& glm::Abs(n1[nSolutions].Dot(n3[nSolutions])) < angleEps
				&& glm::Abs(n2[nSolutions].Dot(n3[nSolutions])) < angleEps)
				++nSolutions;
		}
		if (V2 >= -eps && T2 >= -eps && U2 >= -eps && V2 <= 1.f + eps && T2 <= 1.f + eps && U2 <= 1.f + eps)
		{
			n1[nSolutions] = vec(a + b*T2).Normalized();
			n2[nSolutions] = vec(c + d*U2).Normalized();
			n3[nSolutions] = vec(e + f*V2).Normalized();
			if (glm::Abs(n1[nSolutions].Dot(n2[nSolutions])) < angleEps
				&& glm::Abs(n1[nSolutions].Dot(n3[nSolutions])) < angleEps
				&& glm::Abs(n2[nSolutions].Dot(n3[nSolutions])) < angleEps)
				++nSolutions;
		}
		if (s < 1e-4f && nSolutions == 2)
			 nSolutions = 1;
				
		return nSolutions;
	}
}

// A heuristic(?) that checks of the face normals of two edges are not suitably oriented.
// This is used to skip certain configurations.
static bool AreEdgesBad(const vec &f1a, const vec &f1b, const vec &f2a, const vec &f2b)
{
	MARK_UNUSED(f1a);
	MARK_UNUSED(f1b);
	MARK_UNUSED(f2a);
	MARK_UNUSED(f2b);
	return false;
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
//#define ENABLE_TIMING

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

bool AreCompatibleOpposingEdges(const vec &f1a, const vec &f1b, const vec &f2a, const vec &f2b, vec &outN)
{
	/*
		n1 = f1a*t + f1b*(1-t)
		n2 = f2a*u + f2b*(1-u)
		n1 = -c*n2, where c > 0
		f1a*t + f1b*(1-t) = -c*f2a*u - c*f2b*(1-u)
		f1a*t - f1b*t + cu*f2a + c*f2b - cu*f2b = -f1b
		c*f2b + t*(f1a-f1b) + cu*(f2a-f2b) = -f1b

		M * v = -f1b, where

		M = [ f2b, (f1a-f1b), (f2a-f2b) ] column vectors
		v = [c, t, cu]
	*/

	const float tooCloseToFaceEpsilon = 1e-4f;

	float3x3 A;
	A.SetCol(0, f2b); // c
	A.SetCol(1, (f1a - f1b)); // t
	A.SetCol(2, (f2a - f2b)); // r = c*u
	vec x;
	bool success = A.SolveAxb(-f1b, x);
	float c = x[0];
	float t = x[1];
	float cu = x[2];
	if (!success || c <= 0.f || t < 0.f || t > 1.f)
		return false;
	float u = cu / c;
	if (t < tooCloseToFaceEpsilon || t > 1.f - tooCloseToFaceEpsilon
		|| u < tooCloseToFaceEpsilon || u > 1.f - tooCloseToFaceEpsilon)
		return false;
	if (cu < 0.f || cu > c)
		return false;
	outN = f1b + (f1a-f1b)*t;
	return true;
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

bool SortedArrayContains(const std::vector<int> &arr, int i)
{
	size_t left = 0;
	size_t right = arr.size() - 1;
	if (arr[left] == i || arr[right] == i)
		return true;
	if (arr[left] > i || arr[right] < i)
		return false;

	while(left < right)
	{
		size_t middle = (left + right + 1) >> 1;
		if (arr[middle] < i)
			left = i;
		else if (arr[middle] > i)
			right = i;
		else
			return true;
	}
	return false;
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

void FORCE_INLINE TestThreeAdjacentFaces(const vec &n1, const vec &n2, const vec &n3,
	int edgeI, int edgeJ, int edgeK,
	const Polyhedron &convexHull, const std::vector<std::pair<int, int> > &edges,
	const std::vector<std::vector<int> > &antipodalPointsForEdge,
	float *minVolume, OBB *minOBB)
{
	// Compute the most extreme points in each direction.
	float maxN1 = n1.Dot(convexHull.v[edges[edgeI].first]);
	float maxN2 = n2.Dot(convexHull.v[edges[edgeJ].first]);
	float maxN3 = n3.Dot(convexHull.v[edges[edgeK].first]);
	float minN1 = FLOAT_INF;
	float minN2 = FLOAT_INF;
	float minN3 = FLOAT_INF;
	for(size_t l = 0; l < antipodalPointsForEdge[edgeI].size(); ++l) // O(constant)?
		minN1 = glm::min(minN1, n1.Dot(convexHull.v[antipodalPointsForEdge[edgeI][l]]));
	for(size_t l = 0; l < antipodalPointsForEdge[edgeJ].size(); ++l) // O(constant)?
		minN2 = glm::min(minN2, n2.Dot(convexHull.v[antipodalPointsForEdge[edgeJ][l]]));
	for(size_t l = 0; l < antipodalPointsForEdge[edgeK].size(); ++l) // O(constant)?
		minN3 = glm::min(minN3, n3.Dot(convexHull.v[antipodalPointsForEdge[edgeK][l]]));
	float volume = (maxN1 - minN1) * (maxN2 - minN2) * (maxN3 - minN3);
	if (volume < *minVolume)
	{
		minOBB->axis[0] = n1;
		minOBB->axis[1] = n2;
		minOBB->axis[2] = n3;
		minOBB->r[0] = (maxN1 - minN1) * 0.5f;
		minOBB->r[1] = (maxN2 - minN2) * 0.5f;
		minOBB->r[2] = (maxN3 - minN3) * 0.5f;
		minOBB->pos = (minN1 + minOBB->r[0])*n1 + (minN2 + minOBB->r[1])*n2 + (minN3 + minOBB->r[2])*n3;
		assert(volume > 0.f);
#ifdef OBB_ASSERT_VALIDITY
		OBB o = OBB::FixedOrientationEnclosingOBB((const vec*)&convexHull.v[0], convexHull.v.size(), minOBB->axis[0], minOBB->axis[1]);
		assert2(EqualRel(o.Volume(), volume), o.Volume(), volume);
#endif
		*minVolume = volume;
	}
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

	// Throughout the algorithm, internal edges can all be discarded, so provide a helper macro to test for that.
#define IS_INTERNAL_EDGE(i) (reinterpret_cast<vec*>(&faceNormals[facesForEdge[i].first])->Dot(faceNormals[facesForEdge[i].second]) > 1.f - 1e-4f)

	TIMING_TICK(
		int numInternalEdges = 0;
		for(size_t i = 0; i < edges.size(); ++i)
			if (IS_INTERNAL_EDGE(i))
				++numInternalEdges;
	);
	TIMING("%d/%d (%.2f%%) edges are internal and will be ignored.", numInternalEdges, (int)edges.size(), numInternalEdges * 100.0 / edges.size());

#ifdef OBB_DEBUG_PRINT
	for(size_t i = 0; i < convexHull.v.size(); ++i)
		LOGI("v[%d] = %s", (int)i, vec(convexHull.v[i]).ToString().c_str());
	for(size_t i = 0; i < edges.size(); ++i)
		LOGI("e[%d] = %d->%d", (int)i, edges[i].first, edges[i].second);
#endif

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

	// Stores for each edge index i the complete list of antipodal vertices for that edge.
	std::vector<std::vector<int> > antipodalPointsForEdge(edges.size());

	TIMING_TICK(
		tick_t tz = Clock::Tick();
		unsigned long long numSpatialStrips = 0);

	// The currently best variant for establishing a spatially coherent traversal order.
	std::vector<int> spatialFaceOrder;
	std::vector<int> spatialEdgeOrder;

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
			if (!IS_INTERNAL_EDGE(thisEdge))
				spatialEdgeOrder.push_back(thisEdge);
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

	// Stores a memory of yet unvisited vertices for current graph search.
	std::vector<int> traverseStack;

// Enable this to remove a big chunk of intelligent logic in favor of very naive operation to find antipodal vertices.
// Use for debugging/crossreferencing against the smart version.
#define NAIVE_ANTIPODAL_SEARCH

#ifdef NAIVE_ANTIPODAL_SEARCH
	// A naive O(E*V) algorithm for computing all antipodal points for each edge.
	for (size_t i = 0; i < edges.size(); ++i) // O(E)
	{
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];
		for(size_t j = 0; j < convexHull.v.size(); ++j) // O(V)
			if (IsVertexAntipodalToEdge(convexHull, j, adjacencyData[j], f1a, f1b))
				antipodalPointsForEdge[i].push_back(j);
	}
#endif

	// Since we do several extreme vertex searches, and the search directions have a lot of spatial locality,
	// always start the search for the next extreme vertex from the extreme vertex that was found during the
	// previous iteration for the previous edge. This has been profiled to improve overall performance by as
	// much as 15-25%.
	int startingVertex = 0;

	TIMING_TICK(
		unsigned long long numVertexNeighborSearches = 0;
		unsigned long long numVertexNeighborSearchImprovements = 0;
	);

	TIMING_TICK(
		tick_t t4 = Clock::Tick();
		size_t numTotalAntipodals = 0;
		for (size_t i = 0; i < antipodalPointsForEdge.size(); ++i)
			numTotalAntipodals += antipodalPointsForEdge[i].size();
	);
	TIMING("Antipodalpoints: %f msecs (avg edge has %.3f antipodal points)", Clock::TimespanToMillisecondsF(t3, t4), (float)numTotalAntipodals/edges.size());
	TIMING("%f vertex neighbor searches per edge, %f improvements", (double)numVertexNeighborSearches/edges.size(), (double)numVertexNeighborSearchImprovements/edges.size());

	// Stores for each edge i the list of all sidepodal edge indices j that it can form an OBB with.
	std::vector<std::vector<int> > compatibleEdges(edges.size());

	// Use a O(E*V) data structure for sidepodal vertices.
	unsigned char *sidepodalVertices = new unsigned char[edges.size()*convexHull.v.size()];
	memset(sidepodalVertices, 0, sizeof(unsigned char)*edges.size()*convexHull.v.size());

// Enable this to perform a dumb search for sidepodal edges. Makes sense mostly for debugging the behavior of the smarter version.
#define NAIVE_SIDEPODAL_SEARCH

#ifdef NAIVE_SIDEPODAL_SEARCH
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
				sidepodalVertices[i*convexHull.v.size()+edges[j].first] = 1;
				sidepodalVertices[i*convexHull.v.size()+edges[j].second] = 1;

				if (i != j)
				{
					compatibleEdges[j].push_back(i);
					sidepodalVertices[j*convexHull.v.size()+edges[i].first] = 1;
					sidepodalVertices[j*convexHull.v.size()+edges[i].second] = 1;
				}
			}
	}

#endif

	TIMING_TICK(
		tick_t t5 = Clock::Tick();
		size_t numTotalEdges = 0;
		for(size_t i = 0; i < compatibleEdges.size(); ++i)
			numTotalEdges += compatibleEdges[i].size();
	);
	TIMING("Companionedges: %f msecs (%d edges have on average %d companion edges each)", Clock::TimespanToMillisecondsF(t4, t5), (int)compatibleEdges.size(), (int)(numTotalEdges / compatibleEdges.size()));
	TIMING("%f vertex neighbor searches per edge, %f improvements", (double)numVertexNeighborSearches/edges.size(), (double)numVertexNeighborSearchImprovements/edges.size());
	TIMING_TICK(tick_t ts = Clock::Tick(););
	TIMING_TICK(tick_t tb = Clock::Tick(););
	TIMING("SortCompanionedges: %f msecs", Clock::TimespanToMillisecondsF(ts, tb));
	TIMING_TICK(t5 = Clock::Tick());

	// Take advantage of spatial locality: start the search for the extreme vertex from the extreme vertex
	// that was found during the previous iteration for the previous edge. This speeds up the search since
	// edge directions have some amount of spatial locality and the next extreme vertex is often close
	// to the previous one. Track two hint variables since we are performing extreme vertex searches to
	// two opposing directions at the same time.
	int extremeVertexSearchHint1 = 0;
	int extremeVertexSearchHint2 = 0;
	int extremeVertexSearchHint3 = 0;
	int extremeVertexSearchHint4 = 0;
	int extremeVertexSearchHint1_b = 0;
	int extremeVertexSearchHint2_b = 0;
	int extremeVertexSearchHint3_b = 0;

	TIMING_TICK(
		unsigned long long numConfigsExplored = 0;
		unsigned long long numBootstrapStepsDone = 0;
		unsigned long long numCommonSidepodalStepsDone = 0;
		unsigned long long numEdgeSqrtEdges = 0;
		);

	// Stores a memory of yet unvisited vertices that are common sidepodal vertices to both currently chosen edges for current graph search.
	std::vector<int> traverseStackCommonSidepodals;

//	for(size_t i = 0; i < edges.size(); ++i) // O(|E|)
	for(size_t ii = 0; ii < spatialEdgeOrder.size(); ++ii)
	{
		size_t i = (size_t)spatialEdgeOrder[ii];
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];

		vec deadDirection = (f1a+f1b)*0.5f;

//		vec e1 = (vec(convexHull.v[edges[i].first]) - vec(convexHull.v[edges[i].second])).Normalized();

		const std::vector<int> &compatibleEdgesI = compatibleEdges[i];

		for(size_t j = 0; j < compatibleEdgesI.size(); ++j) // O(sqrt(|E|))?
		{
			int edgeJ = compatibleEdgesI[j];
			if (edgeJ <= (int)i) continue; // Remove symmetry.
			vec f2a = faceNormals[facesForEdge[edgeJ].first];
			vec f2b = faceNormals[facesForEdge[edgeJ].second];
			if (AreEdgesBad(f1a, f1b, f2a, f2b)) continue;
			TIMING_TICK(++numEdgeSqrtEdges);

			vec deadDirection2 = (f2a+f2b)*0.5f;

			vec searchDir = deadDirection.Cross(deadDirection2);
			float len = searchDir.Normalize();
			if (len == 0.f)
			{
				searchDir = f1a.Cross(f2a);
				len = searchDir.Normalize();
				if (len == 0.f)
					searchDir = f1a.Perpendicular();
			}

			CLEAR_GRAPH_SEARCH();
			float dummy;
			extremeVertexSearchHint1 = convexHull.ExtremeVertexConvex(adjacencyData, searchDir, floodFillVisited, floodFillVisitColor, dummy, extremeVertexSearchHint1); // O(log|V|)?
			TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
			TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
			CLEAR_GRAPH_SEARCH();
			extremeVertexSearchHint2 = convexHull.ExtremeVertexConvex(adjacencyData, -searchDir, floodFillVisited, floodFillVisitColor, dummy, extremeVertexSearchHint2); // O(log|V|)?
			TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
			TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);

// Enable new smarter search for sidepodal vertices. Should be always on.
#define SECOND_GUESS_SIDEPODALS

			int secondSearch = -1;
#ifdef SECOND_GUESS_SIDEPODALS
			if (sidepodalVertices[edgeJ*convexHull.v.size()+extremeVertexSearchHint1]) traverseStackCommonSidepodals.push_back(extremeVertexSearchHint1);
			else traverseStack.push_back(extremeVertexSearchHint1);
			if (sidepodalVertices[edgeJ*convexHull.v.size()+extremeVertexSearchHint2]) traverseStackCommonSidepodals.push_back(extremeVertexSearchHint2);
			else secondSearch = extremeVertexSearchHint2;//traverseStack.push_back(extremeVertexSearchHint2);
#else
			traverseStackCommonSidepodals.push_back(extremeVertexSearchHint1);
			traverseStackCommonSidepodals.push_back(extremeVertexSearchHint2);
#endif

			// Bootstrap to a good vertex that is sidepodal to both edges.
			CLEAR_GRAPH_SEARCH();
			while(!traverseStack.empty())
			{
				int v = traverseStack.front();
				traverseStack.erase(traverseStack.begin());
				if (HAVE_VISITED_VERTEX(v))
					continue;
				MARK_VERTEX_VISITED(v);
				TIMING_TICK(++numBootstrapStepsDone);
				const std::vector<int> &n = adjacencyData[v];
				for(size_t k = 0; k < n.size(); ++k)
				{
					int vAdj = n[k];
					if (!HAVE_VISITED_VERTEX(vAdj) && sidepodalVertices[i*convexHull.v.size()+vAdj])
					{
						if (sidepodalVertices[edgeJ*convexHull.v.size()+vAdj])
						{
							traverseStack.clear();
							if (secondSearch != -1)
							{
								traverseStack.push_back(secondSearch);
								secondSearch = -1;
								MARK_VERTEX_VISITED(vAdj);
							}
							traverseStackCommonSidepodals.push_back(vAdj);
							break;
						}
						else
							traverseStack.push_back(vAdj);
					}
				}
			}

			CLEAR_GRAPH_SEARCH();
			while(!traverseStackCommonSidepodals.empty())
			{
				TIMING_TICK(++numCommonSidepodalStepsDone);
				int v = traverseStackCommonSidepodals.back();
				traverseStackCommonSidepodals.pop_back();
				if (HAVE_VISITED_VERTEX(v))
					continue;
				MARK_VERTEX_VISITED(v);
				const std::vector<int> &n = adjacencyData[v];
				for(size_t k = 0; k < n.size(); ++k)
				{
					int vAdj = n[k];
//					int edgeK = vertexPairsToEdges[std::make_pair(v, vAdj)];
					int edgeK = vertexPairsToEdges[v*convexHull.v.size()+vAdj];
					if (IS_INTERNAL_EDGE(edgeK))
						continue; // Edges inside faces with 180 degrees dihedral angles can be ignored.
					if (sidepodalVertices[i*convexHull.v.size()+vAdj]
						&& sidepodalVertices[edgeJ*convexHull.v.size()+vAdj])
					{
						if (!HAVE_VISITED_VERTEX(vAdj))
							traverseStackCommonSidepodals.push_back(vAdj);

						if (edgeJ < edgeK)
						{
							// Test edge triplet i, edgeJ, edgeK.
							vec f3a = faceNormals[facesForEdge[edgeK].first];
							vec f3b = faceNormals[facesForEdge[edgeK].second];

							if (!AreEdgesBad(f1a, f1b, f3a, f3b) && !AreEdgesBad(f2a, f2b, f3a, f3b))
							{
								vec n1[2], n2[2], n3[2];
								TIMING_TICK(++numConfigsExplored);
								int nSolutions = ComputeBasis(f1a, f1b, f2a, f2b, f3a, f3b, n1, n2, n3);
								for(int s = 0; s < nSolutions; ++s) // O(constant), nSolutions == 0, 1 or 2.
								{
									TestThreeAdjacentFaces(n1[s], n2[s], n3[s], (int)i, edgeJ, edgeK, convexHull,
										edges, antipodalPointsForEdge, &minVolume, &minOBB);
								}
							}
						}
					}
				}
			}
		}
	}
	TIMING("Edgetriplets: %llu edgesqrts, %llu bootstraps (%.3f/edgesqrt), %llu sidepodals steps (%.3f/edgesqrt), "
		"#third edges: %llu (%.3f/edgesqrt) #vertex steps: %llu (%.3f/edgesqrt) "
		"#vertex improvements: %.3f/edgesqrt",
		numEdgeSqrtEdges,
		numBootstrapStepsDone, (float)numBootstrapStepsDone/numEdgeSqrtEdges,
		numCommonSidepodalStepsDone, (float)numCommonSidepodalStepsDone/numEdgeSqrtEdges,
		numConfigsExplored, (float)numConfigsExplored/numEdgeSqrtEdges,
		numVertexNeighborSearches, (float)numVertexNeighborSearches/numEdgeSqrtEdges,
		(float)numVertexNeighborSearchImprovements/numEdgeSqrtEdges);

	TIMING_TICK(tick_t t6 = Clock::Tick());
	TIMING("Edgetripletconfigs: %f msecs (%d configs)", Clock::TimespanToMillisecondsF(t5, t6), numConfigsExplored);
	TIMING_TICK(
		t6 = Clock::Tick();
		unsigned long long numTwoOpposingFacesConfigs = 0;
		numVertexNeighborSearches = 0;
		numVertexNeighborSearchImprovements = 0;
		);

	std::vector<int> antipodalEdges;
	VecArray antipodalEdgeNormals;

	// Main algorithm body for finding all search directions where the OBB is flush with the edges of the
	// convex hull from two opposing faces. This is O(E*sqrtE*logV)?
//	for (size_t i = 0; i < edges.size(); ++i) // O(E)
///	{
	for(size_t ii = 0; ii < spatialEdgeOrder.size(); ++ii)
	{
		size_t i = (size_t)spatialEdgeOrder[ii];
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];

		antipodalEdges.clear();
		antipodalEdgeNormals.clear();

		const std::vector<int> &antipodals = antipodalPointsForEdge[i];
		for(size_t j = 0; j < antipodals.size(); ++j) // O(constant)?
		{
			int antipodalVertex = antipodals[j];
			const std::vector<int> &adjacents = adjacencyData[antipodalVertex];
			for(size_t k = 0; k < adjacents.size(); ++k) // O(constant)?
			{
				int vAdj = adjacents[k];
				if (vAdj < antipodalVertex)
					continue; // We search unordered edges, so no need to process edge (v1, v2) and (v2, v1) twice - take the canonical order to be antipodalVertex < vAdj

//				int edge = vertexPairsToEdges[std::make_pair(antipodalVertex, vAdj)];
				int edge = vertexPairsToEdges[antipodalVertex*convexHull.v.size()+vAdj];
				if ((int)i > edge) // We search pairs of edges, so no need to process twice - take the canonical order to be i < edge.
					continue;
				if (IS_INTERNAL_EDGE(edge))
					continue; // Edges inside faces with 180 degrees dihedral angles can be ignored.

				vec f2a = faceNormals[facesForEdge[edge].first];
				vec f2b = faceNormals[facesForEdge[edge].second];

				vec n;
				bool success = AreCompatibleOpposingEdges(f1a, f1b, f2a, f2b, n);
				if (success)
				{
					antipodalEdges.push_back(edge);
					antipodalEdgeNormals.push_back(n.Normalized());
				}
			}
		}

		const std::vector<int> &compatibleEdgesI = compatibleEdges[i];
		for(size_t j = 0; j < compatibleEdgesI.size(); ++j)
//		for(size_t j = 0; j < antipodalEdges.size(); ++j)
		{
			int edgeJ = compatibleEdgesI[j];
//			const std::vector<int> &compatibleEdgesJ = compatibleEdges[edge];
//			n = n.Normalized();
			for(size_t k = 0; k < antipodalEdges.size(); ++k)
			{
				int edgeK = antipodalEdges[k];

				vec n = antipodalEdgeNormals[k];
				float minN1 = n.Dot(convexHull.v[edges[edgeK].first]);
				float maxN1 = n.Dot(convexHull.v[edges[i].first]);

				// Test all mutual compatible edges.
				vec f3a = faceNormals[facesForEdge[edgeJ].first];
				vec f3b = faceNormals[facesForEdge[edgeJ].second];
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
				float num = n.Dot(f3b);
				float denom = n.Dot(f3b-f3a);
				MoveSign(num, denom);

				const float epsilon = 1e-4f;
				if (denom < epsilon)//Equalglm::Abs(denom, 0.f))
				{
					num = glm::EqualAbs(num, 0.f) ? 0.f : -1.f;
					denom = 1.f;
				}

//				if (v >= 0.f - epsilon && v <= 1.f + epsilon)
				if (num >= denom * -epsilon && num <= denom * (1.f + epsilon))
				{
					float v = num / denom;
					vec n3 = vec(f3b + (f3a - f3b) * v).Normalized();
					vec n2 = n3.Cross(n).Normalized();

					float minN2, maxN2;

					CLEAR_GRAPH_SEARCH();
					int hint = convexHull.ExtremeVertexConvex(adjacencyData, n2, floodFillVisited, floodFillVisitColor, maxN2, (k == 0) ? extremeVertexSearchHint1 : extremeVertexSearchHint1_b); // O(logV)?
					if (k == 0) extremeVertexSearchHint1 = extremeVertexSearchHint1_b = hint;
					else extremeVertexSearchHint1_b = hint;
					TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
					TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);

					CLEAR_GRAPH_SEARCH();
					hint = convexHull.ExtremeVertexConvex(adjacencyData, -n2, floodFillVisited, floodFillVisitColor, minN2, (k == 0) ? extremeVertexSearchHint2 : extremeVertexSearchHint2_b); // O(logV)?
					if (k == 0) extremeVertexSearchHint2 = extremeVertexSearchHint2_b = hint;
					else extremeVertexSearchHint2_b = hint;
					TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
					TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
					minN2 = -minN2;

					float maxN3 = n3.Dot(convexHull.v[edges[edgeJ].first]);

					float minN3 = FLOAT_INF;
					const std::vector<int> &antipodalsEdge3 = antipodalPointsForEdge[edgeJ];
					if (antipodalsEdge3.size() < 20) // If there are very few antipodal vertices, do a very tight loop and just iterate over each.
					{
						for(size_t a = 0; a < antipodalsEdge3.size(); ++a)
							minN3 = glm::min(minN3, n3.Dot(convexHull.v[antipodalsEdge3[a]]));
					}
					else
					{
						// Otherwise perform a spatial locality exploiting graph search.
						CLEAR_GRAPH_SEARCH();
						hint = convexHull.ExtremeVertexConvex(adjacencyData, -n3, floodFillVisited, floodFillVisitColor, minN3, (k == 0) ? extremeVertexSearchHint3 : extremeVertexSearchHint3_b); // O(logV)?
						if (k == 0) extremeVertexSearchHint3 = extremeVertexSearchHint3_b = hint;
						else extremeVertexSearchHint3_b = hint;
						TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
						TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
						minN3 = -minN3;
					}

					float volume = (maxN1 - minN1) * (maxN2 - minN2) * (maxN3 - minN3);
					TIMING_TICK(++numTwoOpposingFacesConfigs);
					if (volume < minVolume)
					{
						minOBB.pos = ((minN1 + maxN1) * n + (minN2 + maxN2) * n2 + (minN3 + maxN3) * n3) * 0.5f;
						minOBB.axis[0] = n;
						minOBB.axis[1] = n2;
						minOBB.axis[2] = n3;
						minOBB.r[0] = (maxN1 - minN1) * 0.5f;
						minOBB.r[1] = (maxN2 - minN2) * 0.5f;
						minOBB.r[2] = (maxN3 - minN3) * 0.5f;
#ifdef OBB_ASSERT_VALIDITY
						OBB o = OBB::FixedOrientationEnclosingOBB((const vec*)&convexHull.v[0], convexHull.v.size(), minOBB.axis[0], minOBB.axis[1]);
						assert2(EqualRel(o.Volume(), volume), o.Volume(), volume);
#endif
						minVolume = volume;
					}
				}
			}
		}
	}

	TIMING_TICK(tick_t t7 = Clock::Tick());
	TIMING("Edgepairsforfaces: %f msecs (%llu configs)", Clock::TimespanToMillisecondsF(t6, t7), numTwoOpposingFacesConfigs);
	TIMING("%f vertex neighbor searches per edge, %f improvements", (double)numVertexNeighborSearches/numTwoOpposingFacesConfigs, (double)numVertexNeighborSearchImprovements/numTwoOpposingFacesConfigs);
	TIMING_TICK(
		tick_t t72 = Clock::Tick();
		int numTwoSameFacesConfigs = 0;
		numVertexNeighborSearches = 0;
		numVertexNeighborSearchImprovements = 0;
		);

	// Main algorithm body for computing all search directions where the OBB touches two edges on the same face.
	// This is O(F*sqrtE*logV)?
	for(size_t ii = 0; ii < spatialFaceOrder.size(); ++ii) // O(F)
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

		if (antipodals.size() < 20)
		{
			minN1 = FLOAT_INF;
			for(size_t j = 0; j < antipodals.size(); ++j)
				minN1 = glm::min(minN1, n1.Dot(convexHull.v[antipodals[j]]));
		}
		else
		{
			CLEAR_GRAPH_SEARCH();
			extremeVertexSearchHint4 = convexHull.ExtremeVertexConvex(adjacencyData, -n1, floodFillVisited, floodFillVisitColor, minN1, extremeVertexSearchHint4); // O(logV)?
			minN1 = -minN1;
			TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
			TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
		}

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
#ifdef OBB_ASSERT_VALIDITY
					OBB o = OBB::FixedOrientationEnclosingOBB((const vec*)&convexHull.v[0], convexHull.v.size(), minOBB.axis[0], minOBB.axis[1]);
					assert2(EqualRel(o.Volume(), volume), o.Volume(), volume);
#endif
					minVolume = volume;
				}
			}
		}
	}

	TIMING_TICK(tick_t t8 = Clock::Tick());
	TIMING("Faceconfigs: %f msecs (%d configs, %d faces, %f configs/face)", Clock::TimespanToMillisecondsF(t72, t8), numTwoSameFacesConfigs, (int)spatialFaceOrder.size(), (double)numTwoSameFacesConfigs/spatialFaceOrder.size());
	TIMING("%f vertex neighbor searches per edge, %f improvements", (double)numVertexNeighborSearches/numTwoSameFacesConfigs, (double)numVertexNeighborSearchImprovements/numTwoSameFacesConfigs);

	// The search for edge triplets does not follow cross-product orientation, so
	// fix that up at the very last step, if necessary.
	if (minOBB.axis[0].Cross(minOBB.axis[1]).Dot(minOBB.axis[2]) < 0.f)
		minOBB.axis[2] = -minOBB.axis[2];
#ifdef MATH_VEC_IS_FLOAT4
	minOBB.r.w = 0.f;
	minOBB.pos.w = 1.f;
#endif
	delete[] sidepodalVertices;
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
