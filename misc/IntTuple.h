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
#ifndef INT_TUPLE_H
#define INT_TUPLE_H

//#include <iostream>
#include <vector>

/** @ingroup Utils
 * @brief An integer tuple class.
 *
 * Warning: The constructor IntTuple(int) doesn't resize the tuple, it
 * creates a 1-tuple.
 */
struct IntTuple
{
  typedef std::vector<int> BaseT;
  typedef std::vector<int>::const_iterator const_iterator;
  typedef std::vector<int>::iterator iterator;

  inline IntTuple() {}
  inline IntTuple(int e1):elements(1,e1) {}
  inline IntTuple(int e1,int e2) { set(e1,e2); }
  inline IntTuple(int e1,int e2,int e3) { set(e1,e2,e3); }
  inline IntTuple(const std::vector<int>& t):elements(t) {}
  inline size_t size() const { return elements.size(); }
  inline bool empty() const { return elements.empty(); }
  inline void resize(size_t s) { elements.resize(s); }
  inline int operator[](int i) const { return elements[i]; }
  inline int& operator[](int i) { return elements[i]; }
  inline operator BaseT& () { return elements; }
  inline operator const BaseT& () const { return elements; }
  inline iterator begin() { return elements.begin(); }
  inline const_iterator begin() const { return elements.begin(); }
  inline iterator end() { return elements.end(); }
  inline const_iterator end() const { return elements.end(); }
  inline bool operator < (const IntTuple& t) const { return std::lexicographical_compare(begin(),end(),t.begin(),t.end()); }
  inline bool operator == (const IntTuple& t) const { return elements == t.elements; }
  inline void fill(int val) { std::fill(elements.begin(),elements.end(),val); }
  inline void set(int e1) { elements.resize(1); elements[0]=e1; }
  inline void set(int e1,int e2) { elements.resize(1); elements[0]=e1; elements[1]=e2; }
  inline void set(int e1,int e2,int e3) { elements.resize(1); elements[0]=e1; elements[1]=e2; elements[2]=e3; }
  inline int getIndex(int x) const {
    for(size_t i=0;i<elements.size();i++) if(x==elements[i]) return (int)i;
    return -1;
  }
  inline bool contains(int x,int& index) const { 
    index=getIndex(x);
    return (index>=0);
  }
  inline bool contains(int x) const { 
    int index=getIndex(x);
    return (index>=0);
  }
  inline int count(int x) const { 
    int n=0;
    for(size_t i=0;i<elements.size();i++) if(x==elements[i]) n++;
    return n;
  }
  inline void operator += (int ofs) { for(size_t i=0;i<elements.size();i++) elements[i]+=ofs; }
  inline void operator -= (int ofs) { for(size_t i=0;i<elements.size();i++) elements[i]-=ofs; }

  std::vector<int> elements;
};

#endif
