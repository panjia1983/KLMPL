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
#include "Clip.h"

namespace MathGeometric {

bool ClipLine1D(double q, double p, double& umin, double& umax)
{
   double r;
   if(p<0) {			//entering
     r=-q/p;
     if(r > umax) return false;
     if(r > umin) umin = r;
   }
   else if(p>0) {
     r=-q/p;
     if(r < umin) return false;
     if(r < umax) umax = r;
   }
   else {
     if(q>0) return false;
   }
   return true;
}

bool ClipLine(const Vector2& x, const Vector2& v, const AABB2D& b, double& u1, double& u2)
{
	//for each face, p is dot(v, normal), q is signed dist to plane (dot(v,normal)-offset)
	//normal order: (-1,0), (1,0), (0,-1), (0,1)
	//offset order: -bmin.x, bmax.x, -bmin.y, bmax.y,
	if(ClipLine1D(b.bmin[0] - x[0], -v[0], u1,u2) && ClipLine1D(x[0] - b.bmax[0], v[0], u1,u2)) {
		if(ClipLine1D(b.bmin[1] - x[1], -v[1], u1,u2) && ClipLine1D(x[1] - b.bmax[1], v[1], u1,u2))	{
  			return true;
		}
	}
	return false;
}

} //namespace Math3D
