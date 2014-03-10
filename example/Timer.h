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
#ifndef MY_TIMER_H
#define MY_TIMER_H

#ifdef WIN32
#include <windows.h>
typedef DWORD TimerCounterType;
#else
#include <sys/time.h>
typedef timeval TimerCounterType;
#endif //WIN32

class Timer
{
 public:
  Timer();
  void Reset();

  // Returns elapsed time in milliseconds,seconds respectively
  long long ElapsedTicks();
  double ElapsedTime();

  // Doesn't refresh the current time
  long long LastElapsedTicks() const;
  double LastElapsedTime() const;

 private:
  TimerCounterType start;
  TimerCounterType current;
};

#endif
