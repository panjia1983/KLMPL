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
#include "AABB2D.h"
using namespace MathGeometric;
using namespace ConstantHelper;
AABB2D::AABB2D()
{
}

AABB2D::AABB2D(const Vector2& _bmin,const Vector2& _bmax)
  :bmin(_bmin),bmax(_bmax)
{
}

AABB2D::AABB2D(const AABB2D& rhs)
  :bmin(rhs.bmin),bmax(rhs.bmax)
{
}

bool AABB2D::contains(const Vector2& v) const
{
	return (v.x>=bmin.x && v.x<=bmax.x &&
		v.y>=bmin.y && v.y<=bmax.y);
}

