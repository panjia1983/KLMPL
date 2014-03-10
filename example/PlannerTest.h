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
#ifndef PLANNER_TEST_H
#define PLANNER_TEST_H

#include "MotionPlanning/AnyMotionPlanner.h"
#include "misc/Miscellany.h"

using namespace ConstantHelper;

/** @brief Performs numTrials trials of planning runs on cspace using the 
 * given planner factory.  Optional limits on the # of planning iterations
 * and maximum time.
 */
void PrintPlannerTest(MotionPlannerFactory& factory,CSpace* cspace,
					  const Config& start,const Config& goal,
					  int numTrials,
					  int maxPlanIters=1000,double maxTime=Inf);

#endif
