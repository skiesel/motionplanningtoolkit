#pragma once

#include "../utilities/flannkdtreewrapper.hpp"
#include "../tree_interfaces/treeinterface.hpp"
#include "../samplers/uniformsampler.hpp"
<<<<<<< HEAD
<<<<<<< HEAD
=======
#include "../samplers/goalbiassampler.hpp"
>>>>>>> skiesel/master
=======
#include "../samplers/goalbiassampler.hpp"
>>>>>>> skiesel/master

template<class Workspace, class Agent, class PostProcessor>
class RestartingRRTWithPostProcessing {
public:

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	RestartingRRTWithPostProcessing(const Workspace &workspace, const Agent &agent, const PostProcessor &postProcessor, const InstanceFileMap &args) :
		workspace(workspace), agent(agent), postProcessor(postProcessor), args(args) {}


<<<<<<< HEAD
<<<<<<< HEAD
	std::vector<const Edge*> query(const State &start, const State &goal) {
		typedef flann::KDTreeIndexParams KDTreeType;
		typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, typename Agent::Edge> KDTree;
		typedef UniformSampler<Workspace, Agent, KDTree> Sampler;
		typedef TreeInterface<Agent, KDTree, Sampler> TreeInterface;
		typedef RRT<Workspace, Agent, TreeInterface> Planner;

		std::vector<const Edge*> best;
		double bestCost = std::numeric_limits<double>::infinity();
		while(true) {

			KDTreeType kdtreeType(3);
			KDTree kdtree(kdtreeType, agent.getTreeStateSize());
			Sampler sampler(workspace, agent, kdtree);
			TreeInterface treeInterface(kdtree, sampler);
=======
=======
>>>>>>> skiesel/master
	std::vector<const Edge*> query(const State &start, const State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {
		typedef flann::KDTreeIndexParams KDTreeType;
		typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, typename Agent::Edge> KDTree;
		typedef UniformSampler<Workspace, Agent, KDTree> USampler;
		typedef GoalBiasSampler<Agent, USampler> GBSampler;
		typedef TreeInterface<Agent, KDTree, GBSampler> TreeInterface;
		typedef RRT<Workspace, Agent, TreeInterface> Planner;

		double goalBias = args.exists("Goal Bias") ? args.doubleVal("Goal Bias") : 0;
		dfpair(stdout, "goal bias", "%g", goalBias);

		std::vector<const Edge*> best;
		double bestCost = std::numeric_limits<double>::infinity();

		while(true) {

			KDTreeType kdtreeType(1);
			KDTree kdtree(kdtreeType, agent.getTreeStateSize());
			USampler uniformsampler(workspace, agent, kdtree);

			GBSampler goalbiassampler(uniformsampler, goal, goalBias);

			TreeInterface treeInterface(kdtree, goalbiassampler);
<<<<<<< HEAD
>>>>>>> skiesel/master
=======
>>>>>>> skiesel/master
			Planner planner(workspace, agent, treeInterface, args);

			std::vector<const Edge*> incumbent = planner.query(start, goal);

			incumbent = postProcessor.postProcess(incumbent);

			double incumbentCost = incumbent.back()->gCost();
			if(incumbentCost < bestCost) {
				bestCost = incumbentCost;
				best.clear();
				best.insert(best.begin(), incumbent.begin(), incumbent.end());
			}
		}
		return best;
	}

	void dfpairs() const {}

private:
	const Workspace &workspace;
	const Agent &agent;
	const PostProcessor &postProcessor;
	const InstanceFileMap &args;
};