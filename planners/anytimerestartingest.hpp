#pragma once

#include "../utilities/flannkdtreewrapper.hpp"
#include "../tree_interfaces/treeinterface.hpp"
#include "../planners/est.hpp"
#include "../samplers/uniformsampler.hpp"
#include "../samplers/goalbiassampler.hpp"

template<class Workspace, class Agent>
class AnytimeRestartingEST {
public:

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	AnytimeRestartingEST(const Workspace &workspace, const Agent &agent, const InstanceFileMap &args) :
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
		signal(SIGTERM, AnytimeRestartingEST::cleanup);
		boost::thread timer(AnytimeRestartingEST::timerFunction);

		dfrowhdr(stdout, "solution", 3, "cost", "length", "time");

		std::vector<const Edge*> best;
		double bestCost = std::numeric_limits<double>::infinity();

		while(true) {

			Planner planner(workspace, agent, args, true);

			std::vector<const Edge*> incumbent = planner.query(start, goal, -1, true);

			double incumbentCost = incumbent.back()->gCost();
			if(incumbentCost < bestCost) {
				bestCost = incumbentCost;
				dfrow(stdout, "solution", "gug", bestCost, incumbent.size(), (double)(clock()-startTime) / CLOCKS_PER_SEC);
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

	static AnytimeRestartingEST<Workspace, Agent> *selfPtr;
};

template<class Workspace, class Agent>
AnytimeRestartingEST<Workspace, Agent> *AnytimeRestartingEST<Workspace, Agent>::selfPtr = NULL;
