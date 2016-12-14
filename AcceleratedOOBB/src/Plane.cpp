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

/** @file Plane.cpp
	@author Jukka Jylänki
	@brief Implementation for the Plane geometry object. */
#include "Plane.h"

Plane::Plane(const vec &normal_, float d_)
:normal(normal_), d(d_)
{
}

Plane::Plane(const vec &v1, const vec &v2, const vec &v3)
{
	Set(v1, v2, v3);
}

Plane::Plane(const vec &point, const vec &normal_)
{
	Set(point, normal_);
}

Plane::Plane(const Line &line, const vec &normal)
{
	vec perpNormal = normal - normal.ProjectToNorm(line.dir);
	Set(line.pos, perpNormal.Normalized());
}

void Plane::Set(const vec &v1, const vec &v2, const vec &v3)
{
	normal = glm::cross(v2-v1, v3-v1);
	normal.Normalize();
	d = normal.Dot(v1);
}

void Plane::Set(const vec &point, const vec &normal_)
{
	normal = normal_;
	d = point.Dot(normal);
}

float Plane::SignedDistance(const vec& point) const
{
	return normal.Dot(point) - d;
}

float Plane::Distance(const vec& point) const
{
	return glm::Abs(SignedDistance(point));
}

bool Plane::Contains(const vec& point, float distanceThreshold) const
{
	return Distance(point) <= distanceThreshold;
}
