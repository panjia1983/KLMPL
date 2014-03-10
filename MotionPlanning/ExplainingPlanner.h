#ifndef EXPLAINING_PLANNER_H
#define EXPLAINING_PLANNER_H

#include "MotionPlanner.h"
#include "ExplicitCSpace.h"
#include <map>
#include <vector>
#include <iostream>

struct Subset
{
  Subset(int maxItem=0);
  Subset(const Subset& s);
  Subset(const std::vector<bool>& bits);
  bool operator < (const Subset& s) const;
  bool operator > (const Subset& s) const;
  bool operator == (const Subset& s) const;
  bool operator != (const Subset& s) const;
  Subset operator + (const Subset& s) const;
  Subset operator - () const;
  int count() const;
  double cost(const std::vector<double>& weights) const;
  bool is_subset(const Subset& s) const;

  int maxItem;
  std::set<int> items;
};

std::ostream& operator << (std::ostream& out,const Subset& s);

/** @brief A planner that minimizes the the number of violated constraints
 * using a RRT-like strategy.
 * 
 * Usage:
 *   //first, set up ExplicitCSpace cspace.
 *   ErrorExplainingPlanner planner(&cspace);
 *   planner.Init(start,goal);
 *   
 *   //do planning with a given expansion schedule
 *   vector<int> schedule;
 *   schedule.push_back(limit1);
 *     ...
 *   schedule.push_back(limitN);
 *   Subset cover;
 *   vector<int> bestPlan;
 *   planner.Plan(0,schedule,bestPlan,cover);
 *
 *   //output best path
 *   MilestonePath path;
 *   planner.GetMilestonePath(bestPlan,path);
 */
class ErrorExplainingPlanner
{
 public:
  struct Milestone {
    Config q;
    int mode;
  };
  struct Edge {
    SmartPointer<EdgePlanner> e;
    int mode;
  };
  typedef Graph::UndirectedGraph<Milestone,Edge> Roadmap;

  struct Mode {
    Subset subset;      //subset covered by this mode
    std::vector<int> roadmapNodes;
    std::vector<Subset> pathCovers;   //minimal covers leading from the start to this mode
    double minCost;
  };
  struct Transition {
    std::vector<std::pair<int,int> > connections;
  };
  typedef Graph::UndirectedGraph<Mode,Transition> ModeGraph;

  ErrorExplainingPlanner(ExplicitCSpace* space);
  void Init(const Config& start,const Config& goal);
  ///Performs one iteration of planning given a limit on the explanation size
  void Expand(double maxExplanationCost,vector<int>& newNodes);
  void Expand2(double maxExplanationCost,vector<int>& newNodes);
  ///Performs bottom-up planning according to a given limit expansion schedule
  void Plan(int initialLimit,const vector<int>& expansionSchedule,vector<int>& bestPath,Subset& cover);
  ///Outputs the graph with the given explanation limit
  void BuildRoadmap(double maxExplanationCost,RoadmapPlanner& prm);
  ///Outputs the CC graph.  Each node is a connected component of the roadmap
  ///within the same subset.
  void BuildCCGraph(Graph::UndirectedGraph<Subset,int>& G);
  ///A search that finds a path subject to a coverage constraint
  bool CoveragePath(int s,int t,const Subset& cover,std::vector<int>& path,Subset& pathCover);
  ///A greedy heuristic that performs smallest cover given predecessor
  bool GreedyPath(int s,int t,std::vector<int>& path,Subset& pathCover);
  ///An optimal search
  bool OptimalPath(int s,int t,std::vector<int>& path,Subset& pathCover);
  ///Returns the cover of the path from s->node + completion(node,goal)
  ///where the path cover is determined using the greedy
  ///heuristic
  void Completion(int s,int node,int t,Subset& pathCover);

  //helpers
  int AddNode(const Config& q,int parent=-1);
  int AddNode(const Config& q,const Subset& subset,int parent=-1);
  bool AddEdge(int i,int j,int depth=0);
  int AddEdge(int i,const Config& q,double maxExplanationCost);  //returns index of q
  void AddEdgeRaw(int i,int j);
  int ExtendToward(int i,const Config& qdest,double maxExplanationCost);
  void KNN(const Config& q,int k,vector<int>& neighbors,vector<double>& distances);
  void KNN(const Config& q,double maxExplanationCost,int k,vector<int>& neighbors,vector<double>& distances);
  void UpdatePathsGreedy();
  void UpdatePathsComplete();
  void UpdatePathsGreedy2(int nstart=-1);
  void UpdatePathsComplete2(int nstart=-1);
  //returns true if a path from mode a to mode b can improve the explanation
  //at b
  bool CanImproveConnectivity(const Mode& ma,const Mode& mb,double maxExplanationCost);
  //updates the minCost member of m
  void UpdateMinCost(Mode& m);
  //fast checking of whether the cost of the local constraints at q exceed the
  //given limit
  bool ExceedsCostLimit(const Config& q,double limit,Subset& violations);
  //fast checking of whether the cost of the local constraints violated on 
  //the edge ab exceed the given limit
  bool ExceedsCostLimit(const Config& a,const Config& b,double limit,Subset& violations);

  ///Computes the cover of the path
  void GetCover(const std::vector<int>& path,Subset& cover) const;
  ///Computes the length of the path
  double GetLength(const std::vector<int>& path) const;
  ///Returns the MilestonePath
  void GetMilestonePath(const std::vector<int>& path,MilestonePath& mpath) const;
 
  Config start,goal;
  ExplicitCSpace* space;

  //weighted explanation
  vector<double> obstacleWeights;
 
  //settings for RRT* like expansion
  int numConnections;
  double connectThreshold,expandDistance,goalConnectThreshold;
  double goalBiasProbability;  //probability of expanding toward the goal
  bool bidirectional;        //not functional yet
  
  //settings for path update
  ///If true: use the slower complete, exact cover update.
  ///If false: use the faster greedy one.
  bool updatePathsComplete;
  ///If true: do dynamic shortest paths update
  ///If false: do batch updates when needed
  bool updatePathsDynamic;
  ///For complete planning, keep at most this number of covers 
  int updatePathsMax;

  Roadmap roadmap;
  ModeGraph modeGraph;

  //planning statistics
  int numExpands,numRefinementAttempts,numRefinementSuccesses,numExplorationAttempts,
    numEdgeChecks,numConfigChecks,
    numUpdatePaths,numUpdatePathsIterations;
  double timeNearestNeighbors,timeRefine,timeExplore,timeUpdatePaths,timeOverhead;
};

#endif
