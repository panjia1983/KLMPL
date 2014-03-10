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
#ifndef GEOMETRIC_2D_CSPACE_H
#define GEOMETRIC_2D_CSPACE_H

#include "MotionPlanning/CSpace.h"
#include "misc/Circle2D.h"
#include "misc/Triangle2D.h"
#include "misc/AABB2D.h"
#include <vector>
using namespace MathGeometric;
using namespace std;

class Geometric2DCSpace : public CSpace
{
public:
  Geometric2DCSpace();
  void Add(const Triangle2D& tri);
  void Add(const AABB2D& bbox);
  //void Add(const Polygon2D& poly);
  void Add(const Circle2D& sphere);

  //distance queries
  double ObstacleDistance(const Vector2& x) const;
  bool Overlap(const Circle2D& circle) const;

  virtual void Sample(Config& x);
  virtual void SampleNeighborhood(const Config& c,double r,Config& x);
  virtual bool IsFeasible(const Config& x);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);
  virtual double Distance(const Config& x, const Config& y);
  virtual double ObstacleDistance(const Config& x) { return ObstacleDistance(Vector2(x[0],x[1])); }

  bool euclideanSpace;
  double visibilityEpsilon;
  AABB2D domain;
  vector<AABB2D> aabbs;
  vector<Triangle2D> triangles;
  vector<Circle2D> circles;
};

#endif
