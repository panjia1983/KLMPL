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
#include "Plane2D.h"
namespace MathGeometric{

void Plane2D::setPointNormal(const Point2D& a, const Vector2& n)
{
	normal.setNormalized(n);
	offset = dot(a,normal);
}

double Plane2D::distance(const Point2D& v) const
{
	return dot(v,normal) - offset;
}

void Plane2D::setPoints(const Point2D& a, const Point2D& b)
{
	Vector2 v;
	v.setPerpendicular(b-a);
	setPointNormal(a,v);
}

}
