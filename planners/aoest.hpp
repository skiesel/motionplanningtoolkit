#pragma once

#include "../utilities/flannkdtreewrapper.hpp"
#include "../tree_interfaces/treeinterface.hpp"

template<class Workspace, class Agent>
class AOEST {
public:

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	AOEST(const Workspace &workspace, const Agent &agent, const InstanceFileMap &args) :
		workspace(workspace), agent(agent), args(args) {
			timeout = args.doubleVal("Timeout");

			selfPtr = this;
		}

	static void cleanup(int param) {
		selfPtr->dfpairs();
		dffooter(stdout);
		exit(0);
	}

	static void timerFunction() {
		boost::this_thread::sleep(boost::posix_time::milliseconds(selfPtr->timeout * 1000));
		raise(SIGTERM);
	}

	void query(const State &start, const State &goal, clock_t startT) {
		typedef EST<Workspace, Agent> Planner;

		startTime = startT;
		signal(SIGTERM, AOEST::cleanup);
		boost::thread timer(AOEST::timerFunction);

		double goalBias = args.exists("Goal Bias") ? args.doubleVal("Goal Bias") : 0;
		dfpair(stdout, "goal bias", "%g", goalBias);

		dfrowhdr(stdout, "solution", 3, "cost", "length", "time");

		std::vector<const Edge*> best;
		double bestCost = std::numeric_limits<double>::infinity();

		while(true) {

			Planner planner(workspace, agent, args, true, bestCost);

			std::vector<const Edge*> incumbent = planner.query(start, goal);

			double incumbentCost = incumbent.back()->gCost();

			if(incumbentCost < bestCost) {
				bestCost = incumbentCost;
				dfrow(stdout, "solution", "gug", bestCost, incumbent.size(), (double)(clock()-startTime) / CLOCKS_PER_SEC, 0);
			}
		}
	}

	void dfpairs() const {
		clock_t endTime = clock();
		dfpair(stdout, "total time solving time", "%g", (double)(endTime-startTime) / CLOCKS_PER_SEC);
	}

private:
	const Workspace &workspace;
	const Agent &agent;
	const InstanceFileMap &args;
	double timeout;
	clock_t startTime;

	static AOEST<Workspace, Agent> *selfPtr;
};

template<class Workspace, class Agent>
AOEST<Workspace, Agent> *AOEST<Workspace, Agent>::selfPtr = NULL;