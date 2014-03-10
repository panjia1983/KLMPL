#ifndef ROBOTICS_EXPLICIT_CSPACE_H
#define ROBOTICS_EXPLICIT_CSPACE_H

#include "CSpace.h"
#include "EdgePlanner.h"
#include <vector>
#include <string>

class ExplicitEdgePlanner;

/** @ingroup MotionPlanning
 * @brief Configuration space that exposes N obstacle checks.  This
 * functionality can be used in a motion planner to speed up planning by
 * delaying certain expensive obstacle checks, or to provide reasons for
 * infeasibility.
 *
 * Subclasses must implement NumObstacles(), IsFeasible(q,i), and
 * LocalPlanner(a,b,i) at a minimum. 
 * Many planners will also expect subclasses to implement LocalPlanner(a,b)
 * to return a subclass of ExplicitEdgePlanner.  The default implementation
 * of ExplicitEdgePlanner should suffice as long as the obstacle-specific
 * LocalPlanner(a,b,i) is implemented.  In other words,
 * "return new ExplicitEdgePlanner(this,a,b);"
 */
class ExplicitCSpace : public CSpace
{
public:
  virtual ~ExplicitCSpace() {}
  ///Implement this: single-obstacle feasibility check
  virtual bool IsFeasible(const Config&,int obstacle)=0;
  ///Implement this (optional): all obstacle local planner
    virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);
  ///Implement this: single-obstacle local planner
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b,int obstacle) =0;
  ///Implement this: returns the number of obstacles
  virtual int NumObstacles()=0;
  ///Implement this: returns the name of obstacle
  virtual std::string ObstacleName(int obstacle);
  ///Optional: for local planners using obstacle distance
  virtual double ObstacleDistance(const Config& a,int obstacle) { return ConstantHelper::Inf; }

  ///Default implementation runs IsFeasible(q,index) in order until one is
  ///found false
  virtual bool IsFeasible(const Config&);
  ///Returns a vector indicating which obstacles are violated
  virtual void CheckObstacles(const Config&,std::vector<bool>& infeasible);
};

/** @brief Converges an ExplicitCSpace to a regular CSpace based on one
 * selected obstacle.
 */
class SingleObstacleCSpace : public CSpace
{
public:
  SingleObstacleCSpace(ExplicitCSpace* baseSpace,int obstacle);
  virtual ~SingleObstacleCSpace() {}

  virtual bool IsFeasible(const Config& q) { return baseSpace->IsFeasible(q,obstacle); }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) { return baseSpace->LocalPlanner(a,b,obstacle); }
  virtual void Sample(Config& x) { baseSpace->Sample(x); }
  virtual void SampleNeighborhood(const Config& c,double r,Config& x) { baseSpace->SampleNeighborhood(c,r,x); }
  virtual double Distance(const Config& x, const Config& y) { return baseSpace->Distance(x,y); }
  virtual void Interpolate(const Config& x,const Config& y,double u,Config& out) { baseSpace->Interpolate(x,y,u,out); }

  ExplicitCSpace* baseSpace;
  int obstacle;
};

/** @brief Extracts a subset of obstacles from an ExplicitCSpace.
 */
class SubsetExplicitCSpace : public ExplicitCSpace
{
public:
  SubsetExplicitCSpace(ExplicitCSpace* baseSpace);
  SubsetExplicitCSpace(ExplicitCSpace* baseSpace,int obstacle);
  virtual ~SubsetExplicitCSpace() {}
  ///Turn on all constraints
  void EnableAll();
  ///Turn off all constraints
  void EnableNone();

  virtual bool IsFeasible(const Config&,int obstacle);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b,int obstacle);
  virtual int NumObstacles() { return (int)activeSubset.size(); }
  virtual std::string ObstacleName(int obstacle);
  virtual double ObstacleDistance(const Config& a,int obstacle);

  virtual void Sample(Config& x) { baseSpace->Sample(x); }
  virtual void SampleNeighborhood(const Config& c,double r,Config& x) { baseSpace->SampleNeighborhood(c,r,x); }
  virtual double Distance(const Config& x, const Config& y) { return baseSpace->Distance(x,y); }
  virtual void Interpolate(const Config& x,const Config& y,double u,Config& out) { baseSpace->Interpolate(x,y,u,out); }

  ExplicitCSpace* baseSpace;
  std::vector<int> activeSubset;
};

/** @brief Groups together multiple obstacles from an ExplicitCSpace.
 */
class SubgroupExplicitCSpace : public ExplicitCSpace
{
public:
  SubgroupExplicitCSpace(ExplicitCSpace* baseSpace);
  virtual ~SubgroupExplicitCSpace() {}
  void Ungroup();
  void Group(int i,int j);

  virtual bool IsFeasible(const Config&,int obstacle);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b,int obstacle);
  virtual int NumObstacles() { return (int)groups.size(); }
  virtual std::string ObstacleName(int obstacle) { return groupNames[obstacle]; }
  virtual double ObstacleDistance(const Config& a,int obstacle);

  virtual void Sample(Config& x) { baseSpace->Sample(x); }
  virtual void SampleNeighborhood(const Config& c,double r,Config& x) { baseSpace->SampleNeighborhood(c,r,x); }
  virtual double Distance(const Config& x, const Config& y) { return baseSpace->Distance(x,y); }
  virtual void Interpolate(const Config& x,const Config& y,double u,Config& out) { baseSpace->Interpolate(x,y,u,out); }

  ExplicitCSpace* baseSpace;
  std::vector<std::vector<int> > groups;
  std::vector<std::string> groupNames;
};


/** @brief Default edge planner: checks each constraint sequentially using
 * the space's LocalPlanner(a,b,i) method.
 */
class ExplicitEdgePlanner : public EdgePlanner
{
public:
  ExplicitEdgePlanner(ExplicitCSpace* space,const Config& a,const Config& b,bool init=true);
  virtual ~ExplicitEdgePlanner() {}
  virtual bool IsVisible();
  virtual bool IsVisible(int i);
  virtual void CheckVisible(std::vector<bool>& visible);
  virtual void Eval(double u,Config& x) const { space->Interpolate(a,b,u,x); }
  virtual const Config& Start() const { return a; }
  virtual const Config& Goal() const { return b; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const;
  virtual EdgePlanner* ReverseCopy() const;

  //for incremental planners
  bool PlanAll();
  virtual double Priority() const;
  virtual bool Plan();
  virtual bool Done() const;
  virtual bool Failed() const;
  virtual double Priority(int i) const;
  virtual bool Plan(int i);
  virtual bool Done(int i) const;
  virtual bool Failed(int i) const;

  ExplicitCSpace* space;
  Config a,b;
  std::vector<SmartPointer<EdgePlanner> > obsPlanners;
};

class BisectionEpsilonExplicitEdgePlanner : public ExplicitEdgePlanner
{
public:
  BisectionEpsilonExplicitEdgePlanner(ExplicitCSpace* space,const Config& a,const Config& b,double eps);
  virtual ~BisectionEpsilonExplicitEdgePlanner() {}
  virtual bool IsVisible();
  virtual bool IsVisible(int i);
  virtual void CheckVisible(std::vector<bool>& visible);
  virtual EdgePlanner* Copy() const;
  virtual EdgePlanner* ReverseCopy() const;

  bool PlanAll();
  virtual double Priority() const;
  virtual bool Plan();
  virtual bool Done() const;
  virtual bool Failed() const;
  virtual double Priority(int i) const;
  virtual bool Plan(int i);
  virtual bool Done(int i) const;
  virtual bool Failed(int i) const;

 private:
  struct Segment
  {
    inline bool operator < (const Segment& s) const { return length<s.length; }

    std::list<Config>::iterator prev;
    double length;
  };

  std::list<Config> path;
  double epsilon;
  std::priority_queue<Segment,std::vector<Segment> > q;
  std::vector<bool> foundInfeasible;
  Config x;
};

/** @brief Convenience class for edge planner that holds a smart pointer
 * to a CSpace.  Typically used for single-obstacle edge checkers as follows:
 *
 * SingleObstacleCSpace* ospace = new SingleObstacleCSpace(this,obstacle)
 * return new EdgePlannerWithCSpaceContainer(ospace,new XEdgePlanner(ospace,a,b))
 */
class EdgePlannerWithCSpaceContainer : public PiggybackEdgePlanner
{
public:
  EdgePlannerWithCSpaceContainer(SmartPointer<CSpace> space,const SmartPointer<EdgePlanner>& e);
  virtual ~EdgePlannerWithCSpaceContainer() { }
  virtual EdgePlanner* Copy() const;
  virtual EdgePlanner* ReverseCopy() const;

  SmartPointer<CSpace> spacePtr;
};


#endif
