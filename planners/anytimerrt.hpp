#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 
#include "../utilities/datafile.hpp"
#include <signal.h>


template<class Workspace, class Agent, class TreeInterface>
class AnytimeRRT {
public:

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	AnytimeRRT(const Workspace &workspace, const Agent &agent, TreeInterface &treeInterface, const InstanceFileMap &args) :
		workspace(workspace), agent(agent), treeInterface(treeInterface), solutionCost(-1),
		samplesGenerated(0), edgesAdded(0), edgesRejected(0) {
		steeringDT = args.doubleVal("Steering Delta t");
		collisionCheckDT = args.doubleVal("Collision Check Delta t");
		timeout = args.doubleVal("Timeout");

		dfpair(stdout, "steering dt", "%g", steeringDT);
		dfpair(stdout, "collision check dt", "%g", collisionCheckDT);

		selfPtr = this;
	}

	static void cleanup(int param) {
		AnytimeRRT::selfPtr->dfpairs();
		dffooter(stdout);
		exit(0);
	}

	static void timerFunction() {
		boost::this_thread::sleep(boost::posix_time::milliseconds(selfPtr->timeout * 1000));
		raise(SIGTERM);
	}


	void query(const State &start, const State &goal, clock_t startT) {
		startTime = startT;
		signal(SIGTERM, AnytimeRRT::cleanup);
		boost::thread timer(AnytimeRRT::timerFunction);

		dfrowhdr(stdout, "solution", 3, "cost", "length", "time");

		if(agent.isGoal(start, goal)) {
			dfrow(stdout, "solution", "gug", 0, 0, (double)(clock()-startTime) / CLOCKS_PER_SEC);
		}
		
		auto root = pool.construct(start);
		treeInterface.insertIntoTree(root);

		while(true) {

			std::pair<Edge*, State> treeSample = treeInterface.getTreeSample();
			samplesGenerated++;

			auto edge = agent.steer(treeSample.first->end, treeSample.second, steeringDT);


			if(!workspace.safeEdge(agent, edge, collisionCheckDT)) {
				edgesRejected++;
				continue;
			}

			edgesAdded++;

			Edge *e = pool.construct(edge);
			e->updateParent(treeSample.first);

			if(agent.isGoal(e->end, goal)) {
				if(solutionCost < 0 || e->gCost() < solutionCost) {
					solutionCost = e->gCost();
					Edge *cur = e;
					unsigned int edgeCount = 1;
					while(cur->parent != NULL) {
						cur = cur->parent;
						edgeCount++;
					}
					dfrow(stdout, "solution", "gug", e->gCost(), edgeCount, (double)(clock()-startTime) / CLOCKS_PER_SEC);
				}
			}

			bool addedToTree = treeInterface.insertIntoTree(e);

			if(!addedToTree) {
				pool.destroy(e);
			}
		}
	}

	void dfpairs() const {
		dfpair(stdout, "samples generated", "%u", samplesGenerated);
		dfpair(stdout, "edges added", "%u", edgesAdded);
		dfpair(stdout, "edges rejected", "%u", edgesRejected);
		clock_t endTime = clock();
		dfpair(stdout, "total time solving time", "%g", (double)(endTime-startTime) / CLOCKS_PER_SEC);
	}

private:
	const Workspace &workspace;
	const Agent &agent;
	TreeInterface &treeInterface;
	boost::object_pool<Edge> pool;
	double solutionCost;
	double steeringDT, collisionCheckDT, timeout;

	unsigned int samplesGenerated, edgesAdded, edgesRejected;

	clock_t startTime;

	static AnytimeRRT<Workspace, Agent, TreeInterface> *selfPtr;
};

template<class Workspace, class Agent, class TreeInterface>
AnytimeRRT<Workspace, Agent, TreeInterface> *AnytimeRRT<Workspace, Agent, TreeInterface>::selfPtr = NULL;