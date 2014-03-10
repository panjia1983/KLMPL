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
#include "Primitives.h"

using namespace ConstantHelper;

namespace MathGeometric {
Vector2::Vector2()
{}

Vector2::Vector2(const Vector2& v)
:x(v.x), y(v.y)
{}

Vector2::Vector2(double _x)
:x(_x), y(_x)
{}

Vector2::Vector2(double _x, double _y)
:x(_x), y(_y)
{}

Vector2::Vector2(const double data[2])
:x(data[0]), y(data[1])
{}






std::ostream& operator << (std::ostream& out, const Vector2& v)
{
	out << v.x << " " << v.y;
	return out;
}

std::istream& operator >> (std::istream& in, Vector2& v)
{
	in >> v.x >> v.y;
	return in;
}


}
