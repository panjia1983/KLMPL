Minimum Constraint Removal problem README

Kris Hauser
5/9/2012

This file describes how to set up a minimum constraint removal (MCR) problem
in LMPL.  For more information about these problems, please see the paper:

   K. Hauser.  The Minimum Constraint Removal Problem with Three Robotics
   Applications.  In Workshop on the Algorithmic Foundations of Robotics,
   2012.

*** Setting up C-space callbacks ***

First, you will need to define hooks into your own problem that can be used
in the planner.  This requires subclassing the ExplicitCSpace class, found in
MotionPlanning/ExplicitCSpace.h.  This class exposes the individual 
constraint tests for a configuration space.  You will have to override several
methods to customize it to your own space.

The methods that must be overloaded are IsFeasible(q,obs),
LocalPlanner(a,b,obs), NumObstacles(), and optionally ObstacleName(obs).
Obstacles are indexed from 0 to NumObstacles()-1
IsFeasible performs a single-obstacle feasibility check.  LocalPlanner
performs a single-obstacle visibility check.  ObstacleName returns a string
that is, by default, "Obs[x]" where x is the obstacle index.  The method may
be overloaded for printing more informative names.
The arguments q,a,b have type Config, which is simply a typedef to
vector<double>.

ExplicitCSpace itself is a subclass of CSpace, so some methods of the
underlying CSpace may need to be overloaded as well.  In particular, Sample
must be overloaded (generate random samples from the space), and optionally
the SampleNeighborhood, Distance, and Interpolate functions if you are not
dealing with a Euclidean space.

*** Running the planner ***

The planner class is ErrorExplainingPlanner and is found in
MotionPlanning/ExplainingPlanner.h.  It performs a PRM-like construction
limited to a certain maximum explanation size.  The variant of PRM actually
uses an RRT-like expansion so it maintains a connected roadmap and diffuses
quickly across configuration space.  It progressively searches larger
explanation sizes as it goes, but also keeps the limit bounded by the best
explanation found so far so that the explanation only improves over time.

Code for running the planner looks like this:

  /* Set up space here */
  MyExplicitCSpace myspace(myargs);

  /* Set up planner and set parameters (default values shown here) */
  ErrorExplainingPlanner planner(&myspace);
  planner.numConnections = 10;        //compute k-connected PRM
  planner.connectThreshold = Inf;     //haven't tested this setting much
  planner.expandDistance = 0.1;       //how far to expand the PRM toward a random configuration at each iteration
  planner.goalConnectThreshold = 0.5; //distance at which the planner attempts to connect configurations directly to the goal
  planner.usePathCover = true;        //keep this to true, otherwise performance can be quite bad
  planner.updatePathsComplete = false;//governs whether greedy or complete explanation set updates are used.  Can play with this.
  /* Set up planner */
  planner.Init(start,goal);

  /* Set up an explanation limit expansion schedule, up to 5000 iterations */
  vector<int> schedule(5);
  schedule[0] = 1000;
  schedule[1] = 2000;
  schedule[2] = 3000;
  schedule[3] = 4000;
  schedule[4] = 5000;
  
  /* Start planning */
  vector<int> path;
  Subset cover;
  planner.Plan(0,schedule,path,cover);

  //simple print (integers):
  cout<<"Best cover: "<<cover<<endl;

  //or pretty print (obstacle names):
  cout<<"Best cover:"<<endl;
  for(set<int>::const_iterator i=cover.items.begin();i!=cover.items.end();i++)
    cout<<"  "<<myspace.ObstacleName(*i)<<endl;


*** Parameter settings ***

There are a couple of parameters to play with.  First are the PRM construction
parameters.  By default they are tuned to have good performance on well-scaled
problems (e.g., in which the C-space is scaled approximately to a unit
hypercube).  

The updatePathsComplete member governs how the planner computes the minimal
constraint removal sets to reach each connected component of the roadmap. 
This is performed at each step of the plan (note: reducing the number of calls
is an area of possible future improvement). 
- If true, an optimal discrete MCR solver, which has worst case exponential
  time in the number of obstacles. Outside of pathological cases it seems to
  be a constant factor slower than greedy. 
- If false, it uses a greedy discrete MCR solver, which is fast (polynomial
  time).  But explanation quality in the worst case is arbitrarily bad. In
  typical cases it is near exact.

The expansion schedule is also something to play with.  The way that Plan()
inflates the explanation limit is that every time an expansion is scheduled,
it takes a step from the current limit toward the upper bound established
by the best explanation found so far.  The size of this step is governed
by how many scheduled expansions are remaining.  So if there are 3 expansions
remaining, it will move the limit 1/3 of the way toward the upper limit.  The
planner terminates when a feasible path is found (or more precisely, when a
path is found that violates no fewer constraints than the start and goal
configurations themselves).  It also terminates once the number of iterations
exceeds the last number in the expansion schedule.

The reason that this nonuniform stepping is performed is to achieve some
balance between the budget of planning iterations that you are willing to
perform and the quality of the resulting explanation set.


*** MCR Variants ***

Weighted MCR can be chosen by setting the obstacleWeights member of the
ErrorExplainingPlanner.  This must be set to a vector with size equal to
space.NumObstacles(), in which case each entry gives the cost of removing
the obstacle with the corresponding index, or empty, in which case the
weights are uniform.  The planner accepts infinite weights as well
(set to ConstantHelper::Inf).

A PRM*-like connection strategy, in which a logarithmically increasing
neighborhood radius is used for local connections, can be chosen by setting
numConnections = -1.  This is experimental and has not been tested
thoroughly.  Hypothetically an MCR planner may return asymptotically optimal
paths as well as MCR sets, but more work needs to be done.
