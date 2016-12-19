#pragma once
#ifndef GLM_EXT_H
#define GLM_EXT_H
#include <glm/detail/type_vec3.hpp>
#include <glm/geometric.hpp>
#include <vector>
#include <sstream>

namespace glm
{
	inline float min(float a, float b, float c, float d)
	{
		return glm::min(glm::min(a, b), glm::min(c, d));
	}

	inline float max(float a, float b, float c, float d)
	{
		return glm::max(glm::max(a, b), glm::max(c, d));
	}

	class vec3ext : public vec3
	{
	public:
		vec3ext() :
			vec3()
		{}

		vec3ext(vec3 v) :
			vec3(v)
		{}

		vec3ext(float x, float y, float z) :
			vec3(x, y, z)
		{}

		float dot(const vec3& b) const
		{
			return glm::dot(static_cast<vec3>(*this), b);
		}

		float Dot(const vec3& b) const
		{
			return dot(b);
		}

		vec3ext Normalized() const
		{
			return glm::normalize(static_cast<vec3>(*this));
		}

		float Normalize()
		{
			auto length = glm::length(static_cast<vec3>(*this));
			if (length > 1e-6f)
			{
				*this *= 1.f / length;
				return length;
			}

			set(vec3ext(1.f, 0.f, 0.f)); // We will always produce a normalized vector.
			return 0; // But signal failure, so user knows we have generated an arbitrary normalization.
		}

		vec3ext Cross(const vec3& b) const
		{
			return glm::cross(static_cast<vec3>(*this), b);
		}

		vec3ext Perpendicular(const vec3 &hint = vec3(0, 1, 0), const vec3 &hint2 = vec3(0, 0, 1)) const
		{
			auto v = this->Cross(hint);
			auto len = v.Normalize();
			if (len == 0)
				return hint2;
			else
				return v;
		}

		vec3ext ProjectToNorm(const vec3ext &direction) const
		{
			return direction * this->Dot(direction);
		}

		void set(const vec3ext& v)
		{
			x = v.x;
			y = v.y;
			z = v.z;
		}
	};

	inline float Dot(const vec3ext& a, const vec3ext& b)
	{
		return a.dot(b);
	}

	inline vec3ext Abs(const vec3ext& a)
	{
		return abs(static_cast<vec3>(a));
	}

	inline float Abs(const float& a)
	{
		return abs(a);
	}

	/** Compares the two values for equality, allowing the given amount of absolute error. */
	inline bool EqualAbs(float a, float b, float epsilon = 1e-4f)
	{
		return Abs(a - b) < epsilon;
	}
}

typedef glm::vec3ext vec;
typedef std::vector<glm::vec3ext> VecArray;

std::string formatVec3(const glm::vec3& vec);

#endif