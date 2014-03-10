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
#ifndef MATH3D_MISC_H
#define MATH3D_MISC_H

#include "Primitives.h"
#include "misc/Miscellany.h"
//#include <assert.h>

/** @file math3d/Misc.h
 * @ingroup Math3D
 * @brief Miscellaneous utilities on 3D primitives.
 *
 * Contains predicate tests (IsFinite, IsInf, FuzzyZero), faster comparisons
 * (NormLess, DistanceLess) etc.  The comparisons save a square root.
 */
using namespace ConstantHelper;
using namespace Utilities;

namespace MathGeometric {

template<class V>
inline bool DistanceLEQ(const V& x, const V& y, double d) { return x.distanceSquared(y) <= square(d); }

inline int quadratic(double a, double b, double c, double& x1, double& x2) {
	//printf("quadratic %f %f %f\n", a, b, c);

	if(a == 0)
	{
		if(b == 0)
		{
			if(c == 0)
				return -1;
			return 0;
		}
		x1=-c/b;
		return 1;
	}
	if(c == 0) { //det = b^2
		x1 = 0;
		x2 = -b/a;
		return 2;
	}

	double det = b*b-double(4)*a*c;
	if(det < Zero)
		return 0;
	if(det == Zero) {
		x1 = -b/(2.0*a);
		return 1;
	}
	det = sqrt(det);
	if(b<0) det=-det;
	double q = Half*(det - b);
	x1 = q/a;
	x2 = c/q;
	return 2;
}
/*@}*/

} //namespace Math3D

#endif
