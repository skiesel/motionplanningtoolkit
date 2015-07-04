#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>
#include "../utilities/datafile.hpp"

template<class Workspace, class Agent, class TreeInterface>
class RRTConnect {
public:

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	RRTConnect(const Workspace &workspace, const Agent &agent, TreeInterface &treeInterface, const InstanceFileMap &args) :
		workspace(workspace), agent(agent), treeInterface(treeInterface), solutionCost(-1),
		samplesGenerated(0), edgesAdded(0), edgesRejected(0) {
			steeringDT = stod(args.value("Steering Delta t"));
			collisionCheckDT = stod(args.value("Collision Check Delta t"));

			dfpair(stdout, "steering dt", "%g", steeringDT);
			dfpair(stdout, "collision check dt", "%g", collisionCheckDT);
		}


	void query(const State &start, const State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {

		if(agent.isGoal(start, goal)) {
			fprintf(stderr, "found goal\n");
			return;
		}

		if(firstInvocation) {
			auto root = pool.construct(start);
			treeInterface.insertIntoTree(root);
		}

		unsigned int iterations = 0;

		while(true) {

			auto treeSample = treeInterface.getTreeSample();
			samplesGenerated++;

			auto edge = agent.randomSteer(treeSample, steeringDT);

			while(workspace.safeEdge(agent, edge, collisionCheckDT)) {
				edgesAdded++;
				Edge *e = pool.construct(edge);
				treeInterface.insertIntoTree(e);

				if(agent.isGoal(edge.end, goal)) {
					fprintf(stderr, "found goal\n");
					break;
				}

				edge = agent.steerWithControl(edge.end, edge, steeringDT);
			}

			edgesRejected++;

			++iterations;
			if(iterationsAtATime > 0 && ++iterations > iterationsAtATime) break;
		}

#ifdef VREPPLUGIN
	if(solution.size() > 0) {
		if(agent.validateSolution(solution, goal)) {
			fprintf(stderr, "VALID SOLUTION!\n");
		} else {
			fprintf(stderr, "INVALID SOLUTION!\n");
		}
		agent.animateSolution(solution);
	}
#endif
	}

	void dfpairs() const {
		dfpair(stdout, "samples generated", "%u", samplesGenerated);
		dfpair(stdout, "edges added", "%u", edgesAdded);
		dfpair(stdout, "edges rejected", "%u", edgesRejected);
	}

private:
	const Workspace &workspace;
	const Agent &agent;
	TreeInterface &treeInterface;
	boost::object_pool<Edge> pool;
	std::vector<const Edge*> solution;
	std::vector<State> samples;
	double solutionCost;
	double steeringDT, collisionCheckDT;
	unsigned int samplesGenerated, edgesAdded, edgesRejected;
};