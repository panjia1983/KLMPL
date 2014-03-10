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
#include "Circle2D.h"
#include "Misc.h"
#include "misc/Miscellany.h"

using namespace std;
using namespace ConstantHelper;


namespace MathGeometric {

	double Circle2D::distance(const Point2D& v) const
	{
		return (center-v).norm() - radius;
	}

	bool Circle2D::contains(const Point2D& v) const
	{
		return DistanceLEQ(center,v,radius);
	}

	bool Circle2D::intersects(const Line2D& l, double* t1, double* t2) const
	{
		Vector2 offset=center-l.source;
		double o_o=dot(offset,offset), o_b=dot(offset,l.direction), b_b=dot(l.direction,l.direction);
		//so we know there's a root to |offset-t*b|==r
		//o.o-2t*b.o+t^2*b.b=r^2
		double a,b,c;
		a=b_b;
		b=-Two*o_b;
		c=o_o-radius*radius;
		double x1,x2;
		int res=quadratic(a,b,c,x1,x2);
		if(res<=0) return false;
		if(t1 && t2) {
			*t1=x1;
			*t2=x2;
		}
		return true;
	}
} // namespace MathGeometric
