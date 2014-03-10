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
#ifndef ROBOTICS_MULTI_MODAL_CSPACE_H
#define ROBOTICS_MULTI_MODAL_CSPACE_H

#include "CSpace.h"
#include <misc/UndirectedGraph.h>
#include <vector>

/** @ingroup MotionPlanning
 * @brief Multi-modal configuration space base class.
 *
 * The class must define the mapping from Modes to CSpaces, as well
 * as the CSpaces defining the transition region for each pair of
 * adjacent modes.  They also describe methods that will allow planners
 * to traverse the mode graph.
 *
 * There are three types of mode graphs that can be implemented:
 * 1) discrete and small, 2) discrete and large, or
 * 3) continuous.  The implementations of these types takes on
 * different forms.  (Note that representations 2 and 3 are not yet
 * exploited by any planner implemented in LMPL, but we plan to include 
 * support shortly)
 *
 * For discrete and small mode graphs, the easiest implementation is to
 * explicitly represent the mode graph using the ExplicitMMCSpace class.
 * In this class, you simply fill out the modeGraph structure with the 
 * correct CSpace's along its nodes and edges, and all the rest is done.
 *
 * For discrete and large mode graphs, the graph will be represented
 * implicitly in a search-like framework.  Subclasses should overload the
 * CanEnumAdjacent method such that it returns true.  They should also
 * implement the EnumAdjacent method to return all modes adjacent to
 * the given mode m.  
 *
 * For continuous mode graphs, the graph will be represented by sampling.
 * Subclasses should overload the CanSampleAdjacent method such that it
 * returns true.  They should also implement the SampleAdjacent method
 * to sample a set of modes that are adjacent to the given mode m.
 *
 * The Enum and Sample routines are here for possible future implementations
 * of multi modal planners.
 */
template<class Mode>
class MultiModalCSpace
{
 public:
  virtual ~MultiModalCSpace() {}
  //must be overloaded
  virtual bool IsValid(const Mode& m) { return true; }
  virtual CSpace* GetModeCSpace(const Mode& m) { return NULL; }
  virtual CSpace* GetTransitionCSpace(const Mode& m1,const Mode& m2) { return NULL; }

  //getting modes
  virtual bool CanEnum() const { return false; }
  virtual bool CanSample() const { return false; }
  virtual void Enum(std::vector<Mode>& modes) { fprintf(stderr,"MultiModalCSpace: Cannot enumerate all modes\n"); abort(); }
  virtual void Sample(std::vector<Mode>& modes) { fprintf(stderr,"MultiModalCSpace: Cannot sample modes\n"); abort(); }

  //getting/testing adjacencies
  virtual bool CanEnumAdjacent() const { return false; }
  virtual bool CanSampleAdjacent() const { return false; }
  virtual bool CanTestAdjacent() const { return false; }
  virtual void EnumAdjacent(const Mode& m,std::vector<Mode>& modes) { fprintf(stderr,"MultiModalCSpace: Cannot enumerate adjacent modes\n"); abort(); }
  virtual void SampleAdjacent(const Mode& m,std::vector<Mode>& adj) { fprintf(stderr,"MultiModalCSpace: Cannot sample adjacent modes\n"); abort(); }
  virtual bool TestAdjacent(const Mode& m1,const Mode& m2) { fprintf(stderr,"MultiModalCSpace: Cannot test mode adjacency\n"); abort(); return false; }
};

class ExplicitMMCSpace : public MultiModalCSpace<int>
{
 public:
  typedef int Mode;
  typedef Graph::UndirectedGraph<CSpace*,CSpace*> ModeGraph;

  virtual ~ExplicitMMCSpace() {}
  void DeleteAll();

  //must be overloaded
  virtual bool IsValid(const Mode& m) { return m >= 0 && m < modeGraph.NumNodes(); }
  virtual CSpace* GetModeCSpace(const Mode& m) { return modeGraph.nodes[m]; }
  virtual CSpace* GetTransitionCSpace(const Mode& m1,const Mode& m2) { return *modeGraph.FindEdge(m1,m2); }

  //getting modes
  virtual bool CanEnum() const { return true; }
  virtual bool CanSample() const { return true; }
  virtual void Enum(std::vector<Mode>& modes) { modes.resize(modeGraph.nodes.size()); for(size_t i=0;i<modeGraph.nodes.size();i++) modes[i]=i; }
  virtual void Sample(std::vector<Mode>& modes) { Enum(modes); }

  //getting/testing adjacencies
  virtual bool CanEnumAdjacent() const { return true; }
  virtual bool CanSampleAdjacent() const { return true; }
  virtual bool CanTestAdjacent() const { return true; }
  virtual void EnumAdjacent(const Mode& m,std::vector<Mode>& modes) {
    ModeGraph::Iterator e;
    modes.resize(0);
    for(modeGraph.Begin(m,e);!e.end();++e) modes.push_back(e.target());
  }
  virtual void SampleAdjacent(const Mode& m,std::vector<Mode>& adj) { EnumAdjacent(m,adj); }
  virtual bool TestAdjacent(const Mode& m1,const Mode& m2) { return modeGraph.HasEdge(m1,m2); }

  //must fill out this graph beforehand
  ModeGraph modeGraph;
};


#endif
