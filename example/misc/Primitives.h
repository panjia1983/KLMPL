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
#ifndef MATH3D_PRIMITIVES_H
#define MATH3D_PRIMITIVES_H

//#include <math/math.h>
#include <math.h>
#include <iostream>
#include "misc/Miscellany.h"
//#include <utils.h>

/** @defgroup Math3D
 * @brief Definitions of 3D math classes and functions 
 */

/** @file math3d/Primitives.h
 * @ingroup Math3D
 * @brief Class declarations for useful 3D math types.
 */

/** @ingroup Math3D 
 * @brief Contains all the definitions in the Math3D package.
 */
namespace MathGeometric {

/** @addtogroup Math3D */
/*@{*/

class Vector2;

/** @brief A 2D vector class
 *
 * The elements can be accessed as (x,y), or using square brackets [{0,1}].
 */
class Vector2
{
 public:
  Vector2();
  Vector2(const Vector2&);
  Vector2(double x);
  Vector2(double x, double y);
  Vector2(const double* data);

  inline bool operator == (const Vector2&) const;
  inline bool operator != (const Vector2&) const;
  inline const Vector2& operator = (const Vector2&);
  inline void operator += (const Vector2&);
  inline void operator -= (const Vector2&);
  inline void operator *= (double);
  inline void operator /= (double);
  inline double& operator [] (int);
  inline const double& operator [] (int) const;
  inline operator double* ();
  inline operator const double* () const;

  inline void add(const Vector2& a, const Vector2& b);
  inline void sub(const Vector2& a, const Vector2& b);
  inline void mul(const Vector2& a, double);
  inline void div(const Vector2& a, double);
  inline void madd(const Vector2& b, double);
  inline double dot(const Vector2& a) const;
  inline double cross(const Vector2& a) const;   ///< 2d cross product
  inline double distance(const Vector2& a) const;
  inline double distanceSquared(const Vector2& a) const;
  inline double norm() const;
  inline double normSquared() const;
  inline double length() const;		///< = norm
  inline double minElement(int* index=NULL) const;
  inline double maxElement(int* index=NULL) const;
  inline double minAbsElement(int* index=NULL) const;
  inline double maxAbsElement(int* index=NULL) const;

  inline void set(const Vector2&);
  inline void set(double x);
  inline void set(double x, double y);
  inline void set(const double* data);
  inline void setZero();
  inline void setRotation(double rads);    ///< sets x=cos(rads), y=sin(rads) 
  inline void setPerpendicular(const Vector2&);  ///<sets this to the vector rotated 90 degrees ccw
  inline void setOrthogonal(const Vector2& v) { setPerpendicular(v); }
  inline void setNegative(const Vector2&);
  inline void setNormalized(const Vector2&);
  inline void setProjection(const Vector2&, const Vector2&);  ///< sets this to the projection of a on b
  inline void setMinimum(const Vector2&,const Vector2&);
  inline void setMinimum(const Vector2&);
  inline void setMaximum(const Vector2&,const Vector2&);
  inline void setMaximum(const Vector2&);

  inline void get(Vector2&) const;
  inline void get(double& x, double& y) const;
  inline void get(double data[2]) const;
  inline void getNegative(Vector2&) const;
  inline void getNormalized(Vector2&) const;
  inline void getOrthogonal(Vector2&) const;  ///< calculates an orthogonal vector

  inline void inplaceNegative();
  inline void inplaceMul(double);
  inline void inplaceDiv(double);
  inline void inplaceNormalize();

  inline bool isZero(double eps=ConstantHelper::Zero) const;
  inline bool isEqual(const Vector2&,double eps=ConstantHelper::Zero) const;

  union {
    double data[2];
    struct { double x,y; };
  };
};

inline double dot(const Vector2& a, const Vector2& b);
inline double cross(const Vector2& a, const Vector2& b);
inline void normalize(Vector2& a);
inline Vector2 project(const Vector2& a, const Vector2& b);
inline Vector2 operator - (const Vector2& a);
inline Vector2 operator + (const Vector2& a, const Vector2& b);
inline Vector2 operator - (const Vector2& a, const Vector2& b);
inline Vector2 operator * (const Vector2& a, double b);
inline Vector2 operator * (double a, const Vector2& b);
inline Vector2 operator / (const Vector2& a, double b);

//inlined member functions
inline bool Vector2::operator == (const Vector2& a) const { return a.x == x && a.y == y; }
inline bool Vector2::operator != (const Vector2& a) const { return a.x != x || a.y != y; }
inline const Vector2& Vector2::operator = (const Vector2& v) { set(v); return *this; }
inline void Vector2::operator += (const Vector2& v) { x += v.x; y += v.y; }
inline void Vector2::operator -= (const Vector2& v) { x -= v.x; y -= v.y; }
inline void Vector2::operator *= (double c) { inplaceMul(c); }
inline void Vector2::operator /= (double c) { inplaceDiv(c); }
inline double& Vector2::operator [] (int i) { return data[i]; }
inline const double& Vector2::operator [] (int i) const  { return data[i]; }
inline Vector2::operator double* () { return data; }
inline Vector2::operator const double* () const { return data; }
inline void Vector2::add(const Vector2& a, const Vector2& b) { x=a.x+b.x; y=a.y+b.y; }
inline void Vector2::sub(const Vector2& a, const Vector2& b) { x=a.x-b.x; y=a.y-b.y; }
inline void Vector2::mul(const Vector2& a, double b) { x=a.x*b; y=a.y*b; }
inline void Vector2::div(const Vector2& a, double b) { x=a.x/b; y=a.y/b; }
inline void Vector2::madd(const Vector2& a, double b) { x+=a.x*b; y+=a.y*b; }
inline double Vector2::dot(const Vector2& a) const { return x*a.x + y*a.y; }
inline double Vector2::cross(const Vector2& a) const { return x*a.y - y*a.x; }
inline double Vector2::distance(const Vector2& a) const { return sqrt(distanceSquared(a)); }
inline double Vector2::distanceSquared(const Vector2& a) const { return Utilities::square(x-a.x)+Utilities::square(y-a.y); }
inline double Vector2::minElement(int* index) const {
  if(x <= y) { if(index) *index=0; return x; }
  else { if(index) *index=1; return y; }
}
inline double Vector2::maxElement(int* index) const {
  if(x >= y) { if(index) *index=0; return x; }
  else { if(index) *index=1; return y; }
}
inline double Vector2::minAbsElement(int* index) const {
  if(fabs(x) <= fabs(y)) { if(index) *index=0; return x; }
  else { if(index) *index=1; return y; }
}
inline double Vector2::maxAbsElement(int* index) const {
  if(fabs(x) >= fabs(y)) { if(index) *index=0; return x; }
  else { if(index) *index=1; return y; }
}
inline double Vector2::norm() const { return sqrt(normSquared()); }
inline double Vector2::normSquared() const { return Utilities::square(x)+Utilities::square(y); }
inline double Vector2::length() const { return norm(); }
inline void Vector2::set(const Vector2& a) { set(a.x,a.y); }
inline void Vector2::set(double _x) { x = y = _x; }
inline void Vector2::set(double _x, double _y) { x=_x; y=_y; }
inline void Vector2::set(const double* _data) { if(_data) set(_data[0],_data[1]); else setZero(); }
inline void Vector2::setZero() { x = y = ConstantHelper::Zero; }
inline void Vector2::setRotation(double rads) { set(cos(rads), sin(rads)); }
inline void Vector2::setPerpendicular(const Vector2& v) { set(-v.y, v.x); }
inline void Vector2::setNegative(const Vector2& a) { x=-a.x; y=-a.y; }
inline void Vector2::setNormalized(const Vector2& a) { mul(a,ConstantHelper::PseudoInv(a.norm())); }
inline void Vector2::setProjection(const Vector2& a, const Vector2& b) { mul(b, a.dot(b)/b.dot(b)); }
inline void Vector2::setMinimum(const Vector2& v) { if(v.x<x) x=v.x; if(v.y<y) y=v.y; }
inline void Vector2::setMinimum(const Vector2& a,const Vector2& b) { set(Min(a.x,b.x),Min(a.y,b.y)); }
inline void Vector2::setMaximum(const Vector2& v) { if(v.x>x) x=v.x; if(v.y>y) y=v.y; }
inline void Vector2::setMaximum(const Vector2& a,const Vector2& b) { set(Max(a.x,b.x),Max(a.y,b.y)); }
inline void Vector2::get(Vector2& v) const { get(v.x,v.y); }
inline void Vector2::get(double& _x, double& _y) const { _x=x; _y=y; }
inline void Vector2::get(double _data[2]) const { get(_data[0],_data[1]); }
inline void Vector2::getNegative(Vector2& v) const { v.setNegative(*this); }
inline void Vector2::getNormalized(Vector2& v) const  { v.setNormalized(*this); }
inline void Vector2::getOrthogonal(Vector2& v) const { v.setOrthogonal(*this); }
inline void Vector2::inplaceNegative() { x=-x; y=-y; }
inline void Vector2::inplaceMul(double c) { x*=c; y*=c; }
inline void Vector2::inplaceDiv(double c) { x/=c; y/=c; }
inline void Vector2::inplaceNormalize() { inplaceMul(ConstantHelper::PseudoInv(norm())); }
inline bool Vector2::isZero(double eps) const { return ConstantHelper::FuzzyZero(x,eps)&&ConstantHelper::FuzzyZero(y,eps); }
inline bool Vector2::isEqual(const Vector2&a,double eps) const { return ConstantHelper::FuzzyEquals(x,a.x,eps) && ConstantHelper::FuzzyEquals(y,a.y,eps); }

//Vector2

inline double dot(const Vector2& a, const Vector2& b)
{
  return a.dot(b);
}

inline void normalize(Vector2& a)
{
  a.inplaceNormalize();
}

inline double cross(const Vector2& a, const Vector2& b)
{
  return a.cross(b);
}

inline Vector2 project(const Vector2& a, const Vector2& b)
{
  Vector2 temp;
  temp.setProjection(a,b);
  return temp;
}

inline Vector2 operator - (const Vector2& a)
{
  Vector2 temp;
  temp.setNegative(a);
  return temp;
}

inline Vector2 operator + (const Vector2& a, const Vector2& b)
{
  Vector2 temp;
  temp.add(a,b);
  return temp;
}

inline Vector2 operator - (const Vector2& a, const Vector2& b)
{
  Vector2 temp;
  temp.sub(a,b);
  return temp;
}

inline Vector2 operator * (const Vector2& a, double b)
{
  Vector2 temp;
  temp.mul(a,b);
  return temp;
}

inline Vector2 operator * (double a, const Vector2& b)
{
  Vector2 temp;
  temp.mul(b,a);
  return temp;
}

inline Vector2 operator / (const Vector2& a, double b)
{
  Vector2 temp;
  temp.div(a,b);
  return temp;
}

//IO functions

std::ostream& operator << (std::ostream&, const Vector2&);
std::istream& operator >> (std::istream&, Vector2&);

/*@}*/
} 

#endif
