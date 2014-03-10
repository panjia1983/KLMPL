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
#include "PlannerTest.h"
#include "Timer.h"

void PrintPlannerTest(MotionPlannerFactory& factory,CSpace* cspace,
					  const Config& start,const Config& goal,
					  int numtrial,
					  int maxPlanIters,double maxTime)
{
  if(!cspace->IsFeasible(start)) {
    printf("Warning: start configuration is infeasible\n");
  }
  if(!cspace->IsFeasible(goal)) {
    printf("Warning: goal configuration is infeasible\n");
  }
	double meanTime, minTime, _maxTime;
	vector<bool> solved(numtrial,false);
	vector<int> termIter(numtrial);
	vector<double> planTime(numtrial);
	for(int trial=0;trial<numtrial;trial++) {
		printf("Trial %d... ", trial); fflush(stdout);
		Timer timer;
		MotionPlannerInterface* planner = factory.Create(cspace);
		int mstart=planner->AddMilestone(start);
		int mgoal=planner->AddMilestone(goal);
		int iter=0;
		for(iter=0;iter<maxPlanIters;iter++) {
			if(planner->IsConnected(mstart,mgoal)) {
			  printf("Solved on iteration %d.\n",iter);
				solved[trial]=true;
				break;
			}
			planner->PlanMore();
			if(timer.ElapsedTime() > maxTime) {
			  printf("Timed out at %gs.\n",timer.ElapsedTime());
				break;
			}
		}
		//RoadmapPlanner roadmap(cspace);
		//planner->GetRoadmap(roadmap);
		//cout<<roadmap.roadmap.nodes.size()<<" nodes"<<endl;
		if(!solved[trial]) {
			if(planner->IsConnected(mstart,mgoal)) {
				printf("Solved.\n");
				solved[trial]=true;
			}
			else
			  printf("Timed out on iteration %d.\n",iter);
		}
		termIter[trial]=iter;
		planTime[trial]=timer.ElapsedTime();
		delete planner;
	}
	//print stats
	int numSolved=0;

	for(int trial=0;trial<numtrial;trial++)
		if(solved[trial]) numSolved++;
	printf("Solved %d / %d trial\n",numSolved,numtrial);

	meanTime = 0;
	minTime = planTime[0];
	_maxTime = planTime[0];
	int meanIters = 0, minIters = termIter[0], maxIters = termIter[0];
	for(int trial=0;trial<numtrial;trial++) {
		meanTime += planTime[trial];
		if(planTime[trial] < minTime) minTime = planTime[trial];
		if(planTime[trial] > _maxTime) _maxTime = planTime[trial];
		meanIters += termIter[trial];
		if(termIter[trial] < minIters) minIters = termIter[trial];
		if(termIter[trial] > maxIters) maxIters = termIter[trial];
	}
	printf ("Planning time min %g, max %g, mean %g\n", minTime, _maxTime, meanTime / numtrial);
	printf ("Planning iterations min %d, max %d, mean %g\n", minIters, maxIters, double(meanIters) / numtrial);
}
