/*
This file is part of LMPL.

    LMPL is free software: you can redistribute it and/or modify
    it under the terms of the Lesser GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    LMPL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the Lesser
    GNU General Public License for more details.

    You should have received a copy of the Lesser GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef MATH3D_LINE2D_H
#define MATH3D_LINE2D_H

#include "Point.h"

namespace MathGeometric{

//struct AABB2D;
//struct Segment2D;

/** @ingroup Math3D
 * @brief A 2D line class
 *
 * A redundant representation, using a point s on the line and a direction d.
 * Is parameterized by x = s+t*d for all real t.
 */
struct Line2D
{
	double closestPointParameter(const Point2D& in) const;

  Point2D source;
  Vector2 direction;
};

} //namespace Math3D

#endif
