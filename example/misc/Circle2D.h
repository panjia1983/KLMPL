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
#ifndef MATH3D_CIRCLE2D_H
#define MATH3D_CIRCLE2D_H

#include "Point.h"
#include "Line2D.h"

namespace MathGeometric {

/** @ingroup Math3D
 * @brief A 2D circle class
 *
 * Represented by a center and a radius.
 * 
 * Most methods consider the circle as the solid disk.
 * Methods that use the circle boundary have the prefix "boundary".
 */
struct Circle2D
{
	double distance(const Point2D& v) const;
	bool contains(const Point2D& v) const;
	bool intersects(const Line2D& l, double* t1=NULL, double* t2=NULL) const;

	Point2D center;
	double radius;
};

} //namespace MathGeometric

#endif
