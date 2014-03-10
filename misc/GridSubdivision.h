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
#ifndef GEOMETRY_GRID_SUBDIVISION_H
#define GEOMETRY_GRID_SUBDIVISION_H

#include "misc/Vector.h"
#include "misc/IntTuple.h"
#include <list>
#ifdef WIN32
#include <hash_map>
#include <xhash>
#else
//#ifndef __DEPRECATED
#include <ext/hash_map>
//#else
//#include <unordered_map>
//#endif
#endif

#ifdef WIN32
namespace stdext {
inline size_t hash_value(const IntTuple& x)
{
	const size_t pow = 257;
	size_t res=0;
  int p=1;
  for(size_t i=0;i<x.size();i++) {
    res ^= p*x[i];
    p *= pow;
  }
  return res;
}
} //namespace stdext
#endif

namespace Geometry {


#ifndef WIN32
struct IndexHash
{
  IndexHash(size_t pow=257);
  size_t operator () (const std::vector<int>& x) const;
  size_t pow;
};
#endif

/** @ingroup Geometry
 * @brief A grid with a list of arbitrary objects (given by void pointers)
 */
class GridSubdivision
{
public:
  typedef IntTuple Index;
  typedef std::list<void*> ObjectSet;
  //called once per object in the query range, return false to stop enumerating
  typedef bool (*QueryCallback)(void* obj);

  GridSubdivision(int numDims,double h=1);
  GridSubdivision(const Vector& h);
  size_t GetBucketCount() const { /*return buckets.bucket_count();*/ return buckets.size();} // Nuttapong }
  void SetBucketCount(size_t n) {
#ifdef WIN32
		//buckets.rehash(n);
     //buckets.resize(n);
#else
		buckets.resize(n);
#endif
  }
  void Insert(const Index& i,void* data);
  bool Erase(const Index& i,void* data);
  ObjectSet* GetObjectSet(const Index& i);
  void Clear();

  //returns the index of the point
  void PointToIndex(const Vector& p,Index& i) const;
  //same, but with the local coordinates in the bucket [0,1]^n
  void PointToIndex(const Vector& p,Index& i,Vector& u) const;
  //returns the lower/upper corner of the bucket
  void IndexBucketBounds(const Index& i,Vector& bmin,Vector& bmax) const;

  //returns the min/max indices of all occupied cells
  void GetRange(Index& imin,Index& imax) const;
  //returns the min/max bound of all occupied cells
  void GetRange(Vector& bmin,Vector& bmax) const;

  //range imin to imax
  bool IndexQuery(const Index& imin,const Index& imax,QueryCallback f) const;
  //bounding box from bmin to bmax
  bool BoxQuery(const Vector& bmin,const Vector& bmax,QueryCallback f) const;
  //ball with center c, radius r
  bool BallQuery(const Vector& c,double r,QueryCallback f) const;
  //segment from a to b
  bool SegmentQuery(const Vector& a,const Vector& b,QueryCallback f) const;

  //range imin to imax
  void IndexItems(const Index& imin,const Index& imax,ObjectSet& objs) const;
  //bounding box from bmin to bmax
  void BoxItems(const Vector& bmin,const Vector& bmax,ObjectSet& objs) const;
  //ball with center c, radius r
  void BallItems(const Vector& c,double r,ObjectSet& objs) const;
  //segment from a to b
  void SegmentItems(const Vector& a,const Vector& b,ObjectSet& objs) const;

  Vector h;
#ifdef WIN32
  typedef stdext::hash_map<Index,ObjectSet> HashTable;
#else
  typedef __gnu_cxx::hash_map<Index,ObjectSet,IndexHash> HashTable;
#endif
  HashTable buckets;
};


} //namespace Geometry

#endif
