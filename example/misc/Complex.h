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
#ifndef MATH_COMPLEX_H
#define MATH_COMPLEX_H

#include "misc/Miscellany.h"
#include <iostream>
#include <math.h>

using namespace ConstantHelper;

namespace Math {

	/** @ingroup Math
	 * @brief Complex number class (x + iy).
	 */
	class Complex 
	{
	 public:
	  Complex();
	  Complex(const Complex&);
	  Complex(double x);
	  Complex(double x, double y);

	  inline void mul(const Complex& a, const Complex&);
	  inline void setPolar(double r, double theta);
	  double x, y;
	};

	inline Complex operator * (const Complex& a, const Complex& b)
	{
		Complex temp;
		temp.mul(a,b);
		return temp;
	}
	inline void Complex::mul(const Complex& a, const Complex& b) { double tmp = a.x*b.x - a.y*b.y; y = a.x*b.y + a.y*b.x; x = tmp; };
	inline void Complex::setPolar(double r, double theta) { x=cos(theta)*r; y=sin(theta)*r; }

}
#endif
