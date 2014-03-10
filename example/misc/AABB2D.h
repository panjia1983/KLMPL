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
#ifndef MATH3D_AABB2D_H
#define MATH3D_AABB2D_H

#include "Primitives.h"

namespace MathGeometric {

typedef Vector2 Point2D;

/** @ingroup Math3D
 * @brief A 2D axis-aligned bounding box
 */
struct AABB2D
{
  AABB2D();
  AABB2D(const Vector2& bmin,const Vector2& bmax);
  AABB2D(const AABB2D&);
  bool contains(const Point2D&) const;

  Vector2 bmin, bmax;
};

} //namespace MathGeometric

#endif
