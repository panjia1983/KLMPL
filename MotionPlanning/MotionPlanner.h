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
#ifndef ROBOTICS_MOTION_PLANNER_H
#define ROBOTICS_MOTION_PLANNER_H

#include "misc/Tree.h"
#include "misc/UndirectedGraph.h"
#include "misc/ConnectedComponents.h"
#include "misc/SmartPointer.h"
#include <vector>
#include <list>
#include "CSpace.h"
#include "EdgePlanner.h"
#include "MilestonePath.h"

/** @defgroup MotionPlanning
 * @brief Classes to assist in motion planning.
 */

/** @ingroup MotionPlanning
 * @brief A base roadmap planner class.
 *
 * This class is used essentially as a data structure and helper for more
 * sophisticated roadmap planning algorithms.
 */
class RoadmapPlanner
{
public:
  typedef Graph::UndirectedGraph<Config,SmartPointer<EdgePlanner> > Roadmap;

  RoadmapPlanner(CSpace*);
  virtual ~RoadmapPlanner();
  
  virtual void Cleanup();
  virtual void GenerateConfig(Config& x);
  virtual int AddMilestone(const Config& x);
  virtual int TestAndAddMilestone(const Config& x);
  virtual void ConnectEdge(int i,int j,const SmartPointer<EdgePlanner>& e);
  virtual SmartPointer<EdgePlanner> TestAndConnectEdge(int i,int j);
  virtual bool HasEdge(int i,int j) { return roadmap.FindEdge(i,j)!=NULL; }
  virtual SmartPointer<EdgePlanner> GetEdge(int i,int j) { return *roadmap.FindEdge(i,j); }
  virtual bool AreConnected(int i,int j) { return ccs.SameComponent(i,j); }
  virtual bool AreConnected(int i,int j) const { return ccs.SameComponent(i,j); }
  virtual void ConnectToNeighbors(int i,double connectionThresholdg);
  virtual void ConnectToNearestNeighbors(int i,int k);
  virtual void Generate(int numSamples,double connectionThreshold); 
  virtual void CreatePath(int i,int j,MilestonePath& path);

  CSpace* space;
  Roadmap roadmap;
  Graph::ConnectedComponents ccs;
};


/** @ingroup MotionPlanning
 * @brief A base class to be used for tree-based roadmap planners.
 *
 * connectionThreshold is the minimum distance two nodes must be before
 * a connection may be made between them.  This is infinity by default.
 * If it is infinity, connections are attempted to the closest node in
 * a different component.
 */
class TreeRoadmapPlanner
{
public:
  struct Milestone
  {
    Config x;
    int connectedComponent;
  };
  
  typedef Graph::TreeNode<Milestone,SmartPointer<EdgePlanner> > Node;
  
  TreeRoadmapPlanner(CSpace*);
  virtual ~TreeRoadmapPlanner();
  
  virtual void GenerateConfig(Config& x);
  virtual Node* AddMilestone(const Config& x);
  virtual Node* AddFeasibleMilestone(const Config& x);
  virtual Node* AddInfeasibleMilestone(const Config& x) { return NULL; }
  virtual Node* Extend(); 
  virtual void Cleanup();
  virtual void ConnectToNeighbors(Node*);
  virtual EdgePlanner* TryConnect(Node*,Node*);
  //helpers
  //default implementation is O(n) search
  virtual Node* ClosestMilestone(const Config& x);
  virtual Node* ClosestMilestoneInComponent(int component,const Config& x);
  virtual Node* ClosestMilestoneInSubtree(Node* node,const Config& x);
  Node* TryExtend(Node* n,const Config& x);
  void AttachChild(Node* p, Node* c, EdgePlanner* e);   //c will become a child of p
  void CreatePath(Node* a, Node* b, MilestonePath& path);
  
  CSpace* space;
  std::vector<Node*> connectedComponents;
  double connectionThreshold;
  
  //temporary
  std::vector<Node*> milestones;
  Config x;
};


/** @ingroup MotionPlanning
 * @brief A tree-based randomized planner that extends the roadmap by
 * sampling the neighborhood of existing samples.
 *
 * The existing sample picked is selected with probability proportional
 * to its value in the weight vector.
 * By default, a new node is given weight 1.
 *
 * The EST planner can be implemented on top of this planner by
 * setting appropriate weights.  This functionality is not yet implemented.
 */
class PerturbationTreePlanner : public TreeRoadmapPlanner
{
public:
  PerturbationTreePlanner(CSpace*s);
  virtual void GenerateConfig(Config& x);
  virtual Node* AddFeasibleMilestone(const Config& x); 
  virtual void Cleanup();

  //overrideable 
  virtual Node* SelectMilestone(const std::vector<Node*>& milestones);

  ///Neighborhood distance
  double delta;
  ///Node selection weights
  std::vector<double> weights;
};

/** @ingroup MotionPlanning
 * @brief A basic RRT (Rapidly-Exploring Random Tree) planner.
 *
 * Max distance to expand existing nodes is given in delta.
 *
 * Currently this does not attempt to connect separate trees.
 */
class RRTPlanner : public TreeRoadmapPlanner
{
public:
  RRTPlanner(CSpace*s);
  virtual Node* Extend();
  
  double delta;
};

/** @ingroup MotionPlanning
 * @brief A single-query RRT (Rapidly-Exploring Random Tree) planner.
 *
 * Consists of two trees, one from the start, the other from the goal.
 * Tries to connect the two when an extended config is within
 * connectionThreshold of the other tree.
 *
 * The start and goal configs are stored in milestones 0 and 1, resp.
 */
class BidirectionalRRTPlanner : public RRTPlanner
{
public:
  BidirectionalRRTPlanner(CSpace*s);
  /// Clears the trees, then initializes the start/goal configs
  void Init(const Config& start, const Config& goal);
  /// Performs 1 step of planning, returns true on success
  bool Plan();
  /// Returns the planned path, if successful
  void CreatePath(MilestonePath&) const;
};

/*
class VisibilityPRM : public RandomizedPlanner
{
public:
  VisibilityPRM(CSpace*s);
  //may return NULL for rejected config
  virtual Node* AddMilestone(const Config& x);
  virtual Node* AddInfeasibleMilestone(const Config& x) { return NULL; }
  virtual Node* Extend();
  virtual Node* CanConnectComponent(int i,const Config& x);
};
*/

#endif
