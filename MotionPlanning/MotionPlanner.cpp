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
#include "MotionPlanner.h"
#include "misc/Path.h"
#include "misc/Random.h"
#include "misc/Miscellany.h"
#include <assert.h>

typedef TreeRoadmapPlanner::Node Node;
using namespace std;

//Graph search callbacks

// SetComponentCallback: sets all components to c
struct SetComponentCallback : public Node::Callback
{
  SetComponentCallback(int c) { component = c; }
  virtual void Visit(Node* n) { n->connectedComponent = component; }
  int component;
};

// ClosestMilestoneCallback: finds the closest milestone to x 
struct ClosestMilestoneCallback : public Node::Callback
{
  ClosestMilestoneCallback(CSpace* s,const Config& _x)
	  :space(s),closestDistance(ConstantHelper::Inf),x(_x),closestMilestone(NULL)
  {}
  virtual void Visit(Node* n) {
    double d = space->Distance(x,n->x);
    if(d < closestDistance) {
      closestDistance = d;
      closestMilestone = n;
    }
  }
  CSpace* space;
  double closestDistance;
  const Config& x;
  Node* closestMilestone;
};


TreeRoadmapPlanner::TreeRoadmapPlanner(CSpace* s)
:space(s),connectionThreshold(ConstantHelper::Inf)
{
}

TreeRoadmapPlanner::~TreeRoadmapPlanner()
{
  Cleanup();
}


void TreeRoadmapPlanner::Cleanup()
{
  for(size_t i=0;i<connectedComponents.size();i++)
    SafeDelete(connectedComponents[i]);
  connectedComponents.clear();
  milestones.clear();
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::AddMilestone(const Config& x)
{
  if(space->IsFeasible(x))
    return AddFeasibleMilestone(x);
  else 
    return AddInfeasibleMilestone(x);
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::AddFeasibleMilestone(const Config& x)
{
  Milestone m;
  m.x=x;
  int n=(int)connectedComponents.size();
  m.connectedComponent=n;
  connectedComponents.push_back(new Node(m));
  milestones.push_back(connectedComponents[n]);
  return connectedComponents[n];
}

void TreeRoadmapPlanner::GenerateConfig(Config& x)
{
  space->Sample(x);
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::Extend()
{
  GenerateConfig(x);
  Node* n=AddMilestone(x);
  if(n) ConnectToNeighbors(n);
  return n;
}


void TreeRoadmapPlanner::ConnectToNeighbors(Node* n)
{
  if(n->connectedComponent == -1) return;
  if(ConstantHelper::IsInf(connectionThreshold)==1) {
    //for each other component, attempt a connection to the closest node
    for(size_t i=0;i<connectedComponents.size();i++) {
      if((int)i == n->connectedComponent) continue;
      
      ClosestMilestoneCallback callback(space,n->x);
      connectedComponents[i]->DFS(callback);
      TryConnect(n,callback.closestMilestone);
    }
  }
  else {
    //attempt a connection between this node and all others within the 
    //connection threshold
    for(size_t i=0;i<milestones.size();i++) {
      if(n->connectedComponent != milestones[i]->connectedComponent) {
	if(space->Distance(n->x,milestones[i]->x) < connectionThreshold) {
	  TryConnect(n,milestones[i]);
	}
      }
    }
  }
}

EdgePlanner* TreeRoadmapPlanner::TryConnect(Node* a,Node* b)
{
  EdgePlanner* e=space->LocalPlanner(a->x,b->x);
  if(e->IsVisible()) {
    if(a->connectedComponent < b->connectedComponent) AttachChild(a,b,e);
    else AttachChild(b,a,e);
    return e;
  }
  delete e;
  return NULL;
}

void TreeRoadmapPlanner::AttachChild(Node* p, Node* c, EdgePlanner* e)
{
  assert(p->connectedComponent != c->connectedComponent);
  connectedComponents[c->connectedComponent] = NULL;
  c->reRoot();
  SetComponentCallback callback(p->connectedComponent);
  c->DFS(callback);
  p->addChild(c);
  c->edgeFromParent() = e;
}

void TreeRoadmapPlanner::CreatePath(Node* a, Node* b, MilestonePath& path)
{
  assert(a->connectedComponent == b->connectedComponent);
  assert(a->LCA(b) != NULL);  //make sure they're on same tree?
  a->reRoot();
  connectedComponents[a->connectedComponent] = a;
  assert(b->hasAncestor(a) || b==a);
  assert(a->getParent()==NULL);

  //get path from a to b
  list<Node*> atob;
  while(b) {
    atob.push_front(b);
    assert(b->connectedComponent == a->connectedComponent);
    b = b->getParent();
  }

  path.edges.resize(atob.size()-1);
  int index=0;
  for(list<Node*>::iterator i=++atob.begin();i!=atob.end();i++) {
    b=*i;
    if(b->x == b->edgeFromParent()->Start())
      path.edges[index]=b->edgeFromParent()->ReverseCopy();
    else {
      assert(b->x == b->edgeFromParent()->Goal());
      path.edges[index]=b->edgeFromParent()->Copy();
    }
    index++;
  }
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::ClosestMilestone(const Config& x)
{
  if(milestones.empty()) return NULL;
  double dmin=space->Distance(x,milestones[0]->x);
  Node* n=milestones[0];
  for(size_t i=1;i<milestones.size();i++) {
    double d=space->Distance(x,milestones[i]->x);
    if(d<dmin) {
      dmin=d;
      n=milestones[i];
    }
  }
  return n;
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::ClosestMilestoneInComponent(int component,const Config& x)
{
  ClosestMilestoneCallback callback(space,x);
  connectedComponents[component]->DFS(callback);
  return callback.closestMilestone;
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::ClosestMilestoneInSubtree(Node* node,const Config& x)
{
  ClosestMilestoneCallback callback(space,x);
  node->DFS(callback);
  return callback.closestMilestone;
}

TreeRoadmapPlanner::Node* TreeRoadmapPlanner::TryExtend(Node* n,const Config& x)
{
  if(space->IsFeasible(x)) {
    //connect closest to n
    EdgePlanner* e=space->LocalPlanner(x,n->x);
    if(e->IsVisible()) {
      Node* c=AddFeasibleMilestone(x);
      AttachChild(n,c,e);
      return c;
    }
    else {
      delete e;
    }
  }
  return NULL;
}


RoadmapPlanner::RoadmapPlanner(CSpace* s)
  :space(s)
{
}

RoadmapPlanner::~RoadmapPlanner()
{
	Cleanup();
}


void RoadmapPlanner::Cleanup()
{
  roadmap.Cleanup();
  ccs.Clear();
}

void RoadmapPlanner::GenerateConfig(Config& x)
{
  space->Sample(x);
}

int RoadmapPlanner::AddMilestone(const Config& x)
{
  ccs.AddNode();
  return roadmap.AddNode(x);
}

int RoadmapPlanner::TestAndAddMilestone(const Config& x)
{
  if(!space->IsFeasible(x)) return -1;
  return AddMilestone(x);
}

void RoadmapPlanner::ConnectEdge(int i,int j,const SmartPointer<EdgePlanner>& e)
{
  ccs.AddEdge(i,j);
  roadmap.AddEdge(i,j,e);
}

SmartPointer<EdgePlanner> RoadmapPlanner::TestAndConnectEdge(int i,int j)
{
  SmartPointer<EdgePlanner> e=space->LocalPlanner(roadmap.nodes[i],roadmap.nodes[j]);
  if(e->IsVisible()) {
    ConnectEdge(i,j,e);
    return e;
  }
  else {
    e=NULL;
    return NULL;
  }
}

void RoadmapPlanner::ConnectToNeighbors(int i,double connectionThreshold)
{
  for(size_t j=0;j<roadmap.nodes.size();j++) {
    if(ccs.SameComponent(i,j)) continue;
    if(space->Distance(roadmap.nodes[i],roadmap.nodes[j]) < connectionThreshold) {
      TestAndConnectEdge(i,j);
    }
  }
}

void RoadmapPlanner::ConnectToNearestNeighbors(int i,int k)
{
  if(k <= 0) return;
  set<pair<double,int> > knn;
  pair<double,int> node;
  for(size_t j=0;j<roadmap.nodes.size();j++) {
    if(ccs.SameComponent(i,j)) continue;
    node.first = space->Distance(roadmap.nodes[i],roadmap.nodes[j]);
    node.second = j;
    knn.insert(node);
    if((int)knn.size() > k*4)
      knn.erase(--knn.end());
  }
  int numTests=0;
  for(set<pair<double,int> >::const_iterator j=knn.begin();j!=knn.end();j++) {
    if(ccs.SameComponent(i,j->second)) continue;
    TestAndConnectEdge(i,j->second);
    numTests++;
    if(numTests == k) break;
  }
}

void RoadmapPlanner::Generate(int numSamples,double connectionThreshold)
{
  Config x;
  for(int i=0;i<numSamples;i++) {
    GenerateConfig(x);
    int node=TestAndAddMilestone(x);
    if(node >= 0) ConnectToNeighbors(node,connectionThreshold);
  }
}

void RoadmapPlanner::CreatePath(int i,int j,MilestonePath& path)
{
  assert(ccs.SameComponent(i,j));
  Graph::PathIntCallback callback(roadmap.nodes.size(),j);
  roadmap._BFS(i,callback);
  list<int> nodes;
  Graph::GetAncestorPath(callback.parents,j,i,nodes);
  assert(nodes.front()==i);
  assert(nodes.back()==j);
  path.edges.clear();
  path.edges.reserve(nodes.size());
  for(list<int>::const_iterator p=nodes.begin();p!=--nodes.end();++p) {
    list<int>::const_iterator n=p; ++n;
    SmartPointer<EdgePlanner>* e=roadmap.FindEdge(*p,*n);
    assert(e != NULL);
    if((*e)->Start() == roadmap.nodes[*p])
      path.edges.push_back((*e)->Copy());
    else {
      assert((*e)->Goal() == roadmap.nodes[*p]);
      path.edges.push_back((*e)->ReverseCopy());
    }
  }
  assert(path.IsValid());
}




/*
bool LazyCollisionRP::CheckPath(Node* a, Node* b)
{
  assert(a->connectedComponent == b->connectedComponent);
  a->reRoot();
  connectedComponents[a->connectedComponent] = a;
  Node* n=b;
  while(n != a) {
    Node* p = n->getParent();
    assert(p != NULL);
    if(!space->IsVisible(n->x,p->x)) {
      //split at n
      p->detachChild(n);
      SetComponentCallback callback(connectedComponents.size());
      connectedComponents.push_back(n);
      n->DFS(callback);
      return false;
    }
    n = p;
  }
  return true;
}

RandomizedPlanner::Node* LazyCollisionRP::CanConnectComponent(int i,const Config& x)
{
	ClosestMilestoneCallback callback(space,x);
	connectedComponents[i]->DFS(callback);
	if(callback.closestMilestone) {
		if(callback.closestDistance < connectionThreshold) {
			return callback.closestMilestone;
		}
	}
	return NULL;
}
*/


PerturbationTreePlanner::PerturbationTreePlanner(CSpace*s)
  :TreeRoadmapPlanner(s),delta(1)
{}

void PerturbationTreePlanner::Cleanup()
{
  TreeRoadmapPlanner::Cleanup();
  weights.clear();
}

TreeRoadmapPlanner::Node* PerturbationTreePlanner::AddFeasibleMilestone(const Config& x)
{
  assert(milestones.size() == weights.size());
  Node* n=TreeRoadmapPlanner::AddFeasibleMilestone(x);
  assert(n == milestones.back());
  weights.push_back(1);
  assert(milestones.size() == weights.size());
  return n;
}

void PerturbationTreePlanner::GenerateConfig(Config& x)
{
  if(milestones.empty()) {
    cerr<<"PerturbationTreePlanner::GenerateConfig(): No nodes to choose from!"<<endl;
    space->Sample(x);
  }
  else {
    Node* n = SelectMilestone(milestones);
    space->SampleNeighborhood(n->x,delta,x);
  }
}

TreeRoadmapPlanner::Node* PerturbationTreePlanner::SelectMilestone(const vector<Node*>& milestones)
{
  assert(milestones.size()==weights.size());
  double total=ConstantHelper::Zero;
  for(unsigned int i=0;i<milestones.size();i++) {
    total += weights[i];
  }
  //pick randomly from the total
  double val=RandHelper::randWithRange(ConstantHelper::Zero,total);
  for(unsigned int i=0;i<milestones.size();i++) {
    val -= weights[i];
    if(val<=ConstantHelper::Zero) return milestones[i];
  }
  //shouldn't get here
  assert(false);
  return NULL;
}




RRTPlanner::RRTPlanner(CSpace*s)
  :TreeRoadmapPlanner(s),delta(1)
{}

TreeRoadmapPlanner::Node* RRTPlanner::Extend()
{
  Config dest,x;
  space->Sample(dest);

  //pick closest milestone, step in that direction
  Node* closest=ClosestMilestone(dest);
  double dist=space->Distance(closest->x,dest);
  if(dist > delta)
    space->Interpolate(closest->x,dest,delta/dist,x);
  else
    x=dest;

  return TryExtend(closest,x);
}


BidirectionalRRTPlanner::BidirectionalRRTPlanner(CSpace*s)
  :RRTPlanner(s)
{}

void BidirectionalRRTPlanner::Init(const Config& start, const Config& goal)
{
  Cleanup();
  assert(space->IsFeasible(start));
  assert(space->IsFeasible(goal));
  AddFeasibleMilestone(start);
  AddFeasibleMilestone(goal);
  assert(milestones.size()==2);
  assert(milestones[0]->x == start);
  assert(milestones[1]->x == goal);
  assert(connectedComponents.size()==2);
  assert(connectedComponents[0] == milestones[0]);
  assert(connectedComponents[1] == milestones[1]);
}

bool BidirectionalRRTPlanner::Plan()
{
  //If we've already found a path, return true
  if(milestones[0]->connectedComponent == milestones[1]->connectedComponent)
    return true;

  Node* n=Extend();
  if(!n) return false;

  if(n->connectedComponent == milestones[0]->connectedComponent) {
    //attempt to connect to goal, if the distance is < connectionThreshold
    ClosestMilestoneCallback callback(space,n->x);
    milestones[1]->DFS(callback);
    if(callback.closestDistance < connectionThreshold) {
      if(TryConnect(n,callback.closestMilestone)) //connection successful!
	return true;
    }
  }
  else {
    assert(n->connectedComponent == milestones[1]->connectedComponent);
    //attempt to connect to start, if the distance is < connectionThreshold
    ClosestMilestoneCallback callback(space,n->x);
    milestones[0]->DFS(callback);
    if(callback.closestDistance < connectionThreshold) {
      if(TryConnect(callback.closestMilestone,n)) //connection successful!
	return true;
    }
  }
  return false;
}

void BidirectionalRRTPlanner::CreatePath(MilestonePath& p) const
{
  assert(milestones[0]->connectedComponent == milestones[1]->connectedComponent);
  assert(connectedComponents[0] == milestones[0]);
  list<Node*> path;
  Node* n = milestones[1];
  while(n != milestones[0]) {
    path.push_front(n);
    n = n->getParent();
    assert(n != NULL);
  }
  p.edges.resize(0);
  p.edges.reserve(path.size());
  for(list<Node*>::const_iterator i=path.begin();i!=path.end();i++) {
    Node* n = *i;
    SmartPointer<EdgePlanner> e=n->edgeFromParent();
    if(e->Start() == n->x) {
      p.edges.push_back(e->ReverseCopy());
    }
    else if(e->Goal() == n->x) {
      p.edges.push_back(e);
    }
    else {
      cerr<<"Hmm... edge doesn't have node as its start or its goal!"<<endl;
      abort();
    }
  }
}
