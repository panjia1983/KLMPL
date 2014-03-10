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
#ifndef MATH3D_CLIP_H
#define MATH3D_CLIP_H

#include "Plane2D.h"
#include "AABB2D.h"
#include "misc/Vector.h"

/** @file math3d/Clip.h
 * @ingroup Math3D
 * @brief Functions for clipping lines against geometric primitives
 */

namespace MathGeometric {
  /** @addtogroup Math3D */
  /*@{*/

/** @brief Clipping primitive in 1D. 
 *
 * Given line segment x = q+p*u, u in [umin,umax],
 * clips the range to the constraint x <= 0.
 * 
 * Returns true the resulting range is non-empty.
 */
bool ClipLine1D(double q, double p, double& umin, double& umax);

/// Given line segment x = x0+u, u in [umin,umax],
/// clips the range to the constraint a*x <= b
inline bool ClipLine1D(double x0, double a, double b, double& umin, double& umax)
{
  return ClipLine1D(a*x0-b,a,umin,umax);
}

///Clip a line (x,v) with range [u1,u2] to the plane's negative halfspace
///i.e. the normal points outward.
bool ClipLine(const Vector2& x, const Vector2& v, const Plane2D& b, double& u1, double& u2);
///Clip the line to the interior of the bbox
bool ClipLine(const Vector2& x, const Vector2& v, const AABB2D& b, double& u1, double& u2);
  /*@}*/
} 

#endif
