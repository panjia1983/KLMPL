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
#include "Segment2D.h"
#include "Clip.h"
using namespace std;
using namespace MathGeometric;
bool Segment2D::intersects(const AABB2D& bb) const
{
	double u1=0,u2=1;
	return intersects(bb,u1,u2);
}

bool Segment2D::intersects(const AABB2D& bb, double& u1, double& u2) const
{
	return ClipLine(a, b-a, bb, u1,u2);
}
