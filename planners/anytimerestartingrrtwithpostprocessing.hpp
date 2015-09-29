#pragma once

#include "../utilities/flannkdtreewrapper.hpp"
#include "../tree_interfaces/treeinterface.hpp"
#include "../planners/rrt.hpp"
#include "../samplers/uniformsampler.hpp"
#include "../samplers/goalbiassampler.hpp"

template<class Workspace, class Agent, class PostProcessor>
class AnytimeRestartingRRTWithPostProcessing {
public:

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	AnytimeRestartingRRTWithPostProcessing(const Workspace &workspace, const Agent &agent, const PostProcessor &postProcessor, const InstanceFileMap &args) :
		workspace(workspace), agent(agent), postProcessor(postProcessor), args(args) {
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
		typedef flann::KDTreeIndexParams KDTreeType;
		typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, typename Agent::Edge> KDTree;
		typedef UniformSampler<Workspace, Agent, KDTree> USampler;
		typedef GoalBiasSampler<Agent, USampler> GBSampler;
		typedef TreeInterface<Agent, KDTree, GBSampler> TreeInterface;
		typedef RRT<Workspace, Agent, TreeInterface> Planner;

		startTime = startT;
		signal(SIGTERM, AnytimeRestartingRRTWithPostProcessing::cleanup);
		boost::thread timer(AnytimeRestartingRRTWithPostProcessing::timerFunction);

		double goalBias = args.exists("Goal Bias") ? args.doubleVal("Goal Bias") : 0;
		dfpair(stdout, "goal bias", "%g", goalBias);

		dfrowhdr(stdout, "solution", 4, "cost", "length", "time", "shorcut");

		std::vector<const Edge*> best;
		double bestCost = std::numeric_limits<double>::infinity();

		while(true) {

			KDTreeType kdtreeType(1);
			KDTree kdtree(kdtreeType, agent.getDistanceEvaluator(), agent.getTreeStateSize());
			USampler uniformsampler(workspace, agent, kdtree);

			GBSampler goalbiassampler(uniformsampler, goal, goalBias);

			TreeInterface treeInterface(kdtree, goalbiassampler);
			Planner planner(workspace, agent, treeInterface, args, true);

			std::vector<const Edge*> incumbent = planner.query(start, goal, -1, true);

			double incumbentCost = incumbent.back()->gCost();
			if(incumbentCost < bestCost) {
				bestCost = incumbentCost;
				dfrow(stdout, "solution", "gugu", bestCost, incumbent.size(), (double)(clock()-startTime) / CLOCKS_PER_SEC, 0);
			}

			incumbent = postProcessor.postProcess(incumbent);

			incumbentCost = incumbent.back()->gCost();
			if(incumbentCost < bestCost) {
				bestCost = incumbentCost;
				dfrow(stdout, "solution", "gugu", bestCost, incumbent.size(), (double)(clock()-startTime) / CLOCKS_PER_SEC, 1);
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
	const PostProcessor &postProcessor;
	const InstanceFileMap &args;
	double timeout;
	clock_t startTime;

	static AnytimeRestartingRRTWithPostProcessing<Workspace, Agent, PostProcessor> *selfPtr;
};

template<class Workspace, class Agent, class PostProcessor>
AnytimeRestartingRRTWithPostProcessing<Workspace, Agent, PostProcessor> *AnytimeRestartingRRTWithPostProcessing<Workspace, Agent, PostProcessor>::selfPtr = NULL;