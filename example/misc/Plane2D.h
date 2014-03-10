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
#ifndef MATH3D_PLANE2D_H
#define MATH3D_PLANE2D_H

#include "Point.h"

namespace MathGeometric {

/** @brief A 2D plane class
 * @ingroup Math3D
 *
 * Represents plane with a normal and offset such that x on the plane 
 * satisfy dot(normal,x) = offset.
 */
struct Plane2D
{
	void setPoints(const Point2D& a, const Point2D& b);
	void setPointNormal(const Point2D& a, const Vector2& n);
	double distance(const Point2D& v) const;

	Vector2 normal;
	double offset;
};

} //namespace Math3D

#endif
