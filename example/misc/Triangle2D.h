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
#ifndef MATH3D_TRIANGLE2D_H
#define MATH3D_TRIANGLE2D_H

#include "misc/Point.h"
#include "misc/Segment2D.h"
#include "misc/Plane2D.h"

namespace MathGeometric {

struct Plane2D;
struct Segment2D;
struct Line2D;
struct AABB2D;

/** @ingroup Math3D
 * @brief A 2D triangle class
 *
 * Represented by its vertices a,b,c.
 *
 * Barycentric coordinates (u,v,w) are such that 0 <= u,v,w <= 1
 * and u+v+w = 1.  They parameterize the triangle as x = u*a+v*b+w*c.
 *
 * "Plane" coordinates (p,q) are such that 0 <= p,q and p+q<= 1.
 * They parameterize the triangle as x = a + p*(b-a) + q*(c-a).
 * Barycentric coordinates (u,v,w) = (1-p-q,p,q).
 */
struct Triangle2D
{
  Triangle2D();
  Triangle2D(const Vector2& a,const Vector2& b,const Vector2& c);

  Vector2 planeCoords(const Point2D& x) const;
  Point2D planeCoordsToPoint(const Vector2& pc) const;
  
  Vector2 closestPointCoords(const Point2D& in) const;  ///<returns the plane-coords of the point
  Point2D closestPoint(const Point2D& in) const;
  bool contains(const Point2D& x) const;
  
  bool intersect(const Plane2D&, Segment2D& S) const;

  static bool containsPlaneCoords(const Vector2& pc);
  
  Point2D a,b,c;
};

} //namespace MathGeometric

#endif
