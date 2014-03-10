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
#include "Triangle2D.h"
//#include "geometry2d.h"
#include "misc/Miscellany.h"
#include "misc/Clip.h"
#include <stdio.h>
using namespace MathGeometric;
using namespace ConstantHelper;
using namespace std;

Triangle2D::Triangle2D()
{}

Triangle2D::Triangle2D(const Vector2& _a,const Vector2& _b,const Vector2& _c)
:a(_a),b(_b),c(_c)
{}

Vector2 Triangle2D::planeCoords(const Point2D& x) const
{
	double A,B,C,D,E;
	Vector2 e0=b-a,e1=c-a,x0=a-x;
	A = dot(e0,e0);
	B = dot(e0,e1);
	C = dot(e1,e1);
	D = dot(e0,x0);
	E = dot(e1,x0);
	double det = A*C-B*B;
	if(det == Zero) { //collapsed to a line
		return Vector2(Zero);
	}
	return Vector2((B*E-C*D)/det,(B*D-A*E)/det);
}

Point2D Triangle2D::planeCoordsToPoint(const Vector2& pc) const
{
  Point2D pos=a;
  pos.madd(b-a, pc.x);
  pos.madd(c-a, pc.y); 
  return pos;
}


Vector2 Triangle2D::closestPointCoords(const Point2D& in) const
{
	double A,B,C,D,E;
	Vector2 e1=b-a,e2=c-a,x0=a-in;
	A = dot(e1,e1);
	B = dot(e1,e2);
	C = dot(e2,e2);
	D = dot(e1,x0);
	E = dot(e2,x0);
	double det = A*C-B*B;
	if(det == Zero) { //collapsed to a line
		return Vector2(Zero);
	}

	Vector2 pc (B*E-C*D,B*D-A*E);
	if(pc.x < Zero) {  //check edge 2
		//t = dot(in-a,e2)/dot(e2,e2) = -E/C
		double t;
		if(-E <= Zero) t=Zero;
		else if(-E >= C) t=One;
		else t = -E/C;
		return Vector2(Zero,t);
	}
	else if(pc.y < Zero) { //check edge 1
		//t = dot(in-a,e1)/dot(e1,e1) = -D/A
		double t;
		if(-D <= Zero) t=Zero;
		else if(-D >= A) t=One;
		else t = -D/A;
		return Vector2(t,Zero);
	}
	else if(pc.x+pc.y > det) { //check edge 2
		//t = dot(in-a-e1,e2-e1)/dot(e2-e1,e2-e1)
		//= [dot(in-a,e2) - dot(in-a,e1) - dot(e1,e2) + dot(e1,e1)]/(dot(e2,e2) - 2dot(e2,e1) + dot(e1,e1)]
		//= -(B-A+E-D)/(C-2B+A)
		double numer = -(B-A+E-D);
		double denom = (A-2*B+C);
		double t;
		//t = numer/denom with denom >= 0
		if(numer <= Zero) t=Zero;
		else if(numer >= denom) t=One;
		else t = numer/denom;
		return Vector2(One-t,t);
	}
	else {
		pc /= det;
		return pc;
	}
}

Point2D Triangle2D::closestPoint(const Point2D& in) const
{
  return planeCoordsToPoint(closestPointCoords(in));
}

bool Triangle2D::contains(const Point2D& x) const
{
  return containsPlaneCoords(planeCoords(x));
}

bool Triangle2D::intersect(const Plane2D& P, Segment2D& S) const
{
  double d[2]; const Point2D* p[3] = {&a,&b,&c};
  for(int i=0;i<3;i++) d[i]=P.distance(*p[i]);
  //insertion sort
  for(int i=1;i<3;i++) {
    double di=d[i];
    const Point2D* pi=p[i];
    int j=i;
    for(;j>0;j--) {
      if(d[j-1] <= di) break;
      d[j] = d[j-1];
      p[j] = p[j-1];
    }
    d[j] = di;
    p[j] = pi;
  }
  if(!(d[0] <= d[1] && d[1] <= d[2])) {
    printf ("AAAACK: %f %f %f\n",d[0],d[1],d[2]);
  }
  assert(d[0] <= d[1] && d[1] <= d[2]);

  if(d[0] > Zero) return false;
  if(d[2] < Zero) return false;
  double u;
  if(d[1] <= Zero) { //both 0 and 1 are inside p
    if(d[0] == d[2]) u = 0;
    else u = d[0]/(d[0]-d[2]);
    S.a = (One-u)*(*p[0]) + u*(*p[2]);
    if(d[1] == d[2]) u = 0;
    else u = d[1]/(d[1]-d[2]);
    S.b = (One-u)*(*p[1]) + u*(*p[2]);
  }
  else { //only 0 is inside p
    u = d[0]/(d[0]-d[1]);
    S.a = (One-u)*(*p[0]) + u*(*p[1]);
    u = d[0]/(d[0]-d[2]);
    S.b = (One-u)*(*p[0]) + u*(*p[2]);
  }
  return true;
}


bool Triangle2D::containsPlaneCoords(const Vector2& pc)
{
	return pc.x >= Zero && pc.y >= Zero && pc.x+pc.y <= One;
}
