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
#include "Complex.h"

namespace Math {


Complex::Complex()
{}

Complex::Complex(const Complex& z)
:x(z.x), y(z.y)
{}

Complex::Complex(double _x)
:x(_x), y(Zero)
{}

Complex::Complex(double _x, double _y)
:x(_x), y(_y)
{}

} //namespace Math
