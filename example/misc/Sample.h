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
#ifndef MATH_SAMPLE_H
#define MATH_SAMPLE_H

#include <vector>

/** @file math/Sample.h
 * @brief Functions for random sampling of various sets.
 */

namespace MathSample {
/** @addtogroup Math */
/*@{*/
  
/// Uniform distribution inside a circle with radius r
void SampleDisk(double r, double& x, double& y);

} //namespace Math
/*@}*/

#endif
