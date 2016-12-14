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

/** @file Line.h
	@author Jukka Jylänki
	@brief Implementation for the Line geometry object. */
#pragma once
#ifndef LINE_H
#define LINE_H

#include "glmext.h"

/// A line in 3D space is defined by an origin point and a direction, and extends to infinity in two directions.
class Line
{
public:
	/// Specifies the origin of this line.
	vec pos;

	/// The normalized direction vector of this ray. [similarOverload: pos]
	/** @note For proper functionality, this direction vector needs to always be normalized. If you set to this
		member manually, remember to make sure you only assign normalized direction vectors. */
	vec dir;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members pos and dir are undefined after creating a new Line using this
		default constructor. Remember to assign to them before use.
		@see pos, dir. */
	Line() {}

	/// Constructs a new line by explicitly specifying the member variables.
	/** @param pos The origin position of the line.
		@param dir The direction of the line. This vector must be normalized, this function will not normalize
			the vector for you (for performance reasons).
		@see pos, dir. */
	Line(const vec &pos, const vec &dir);
};

#endif