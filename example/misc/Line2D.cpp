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
#include "Line2D.h"
#include "misc/Miscellany.h"
using namespace ConstantHelper;

namespace MathGeometric{
	double Line2D::closestPointParameter(const Point2D& in) const
	{
		double denom = dot(direction,direction);
		if(denom == Zero) return Zero;
		return dot(in-source,direction)/denom;
	}
}
