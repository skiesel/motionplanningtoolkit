#pragma once

#include <flann/flann.hpp>
#include <unordered_set>
#include <boost/pool/object_pool.hpp>
#include "../utilities/datafile.hpp"

template<class Workspace, class Agent, class TreeInterface>
class ReusableRRT {
public:

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	ReusableRRT(const Workspace &workspace, const Agent &agent, TreeInterface &treeInterface, const InstanceFileMap &args, bool quiet = false) :
		workspace(workspace), agent(agent), treeInterface(treeInterface), solutionCost(-1), poseNumber(-1),
		samplesGenerated(0), edgesAdded(0), edgesRejected(0), quiet(quiet) {
		steeringDT = args.doubleVal("Steering Delta t");
		collisionCheckDT = args.doubleVal("Collision Check Delta t");

		if(!quiet) {
			dfpair(stdout, "steering dt", "%g", steeringDT);
			dfpair(stdout, "collision check dt", "%g", collisionCheckDT);
		}
	}

	std::vector<const Edge*> query(const State &start, const State &goal, double gBound, bool firstInvocation = false) {

		if(firstInvocation) {
			auto root = pool.construct(start);
			treeInterface.insertIntoTree(root);
		}

		if(agent.isGoal(start, goal)) {
			if(!quiet) {
				dfpair(stdout, "solution cost", "0");
				dfpair(stdout, "solution length", "0");
			}
			return std::vector<const Edge*>();
		}

		std::unordered_set<Edge*> removed;
		for(Edge *edge : poolEdges) {
			if(edge->gCost() >= gBound) {
				removed.emplace(edge);
				treeInterface.removeFromTree(edge);
				pool.destroy(edge);
			}
		}

		for(auto edge : removed) {
			poolEdges.erase(edge);
		}

		while(true) {

			std::pair<Edge*, State> treeSample = treeInterface.getTreeSample();
			samplesGenerated++;

			auto edge = agent.steer(treeSample.first->end, treeSample.second, steeringDT);

			edge.updateParent(treeSample.first);

			if(edge.gCost() >= gBound) {
				continue;
			}

			if(!workspace.safeEdge(agent, edge, collisionCheckDT)) {
				edgesRejected++;

				continue;
			}

			edgesAdded++;

			Edge *e = pool.construct(edge);
			e->updateParent(treeSample.first);

			if(agent.isGoal(e->end, goal)) {
				if(solutionCost < 0 || e->gCost() < solutionCost) {
					std::vector<const Edge *> newSolution;
					newSolution.push_back(e);

					unsigned int edgeCount = 1;
					while(newSolution.back()->parent != NULL) {
						edgeCount++;
						newSolution.push_back(newSolution.back()->parent);
					}

					if(!quiet) {
						dfpair(stdout, "solution cost", "%g", e->gCost());
						dfpair(stdout, "solution length", "%u", edgeCount);
					}

					poseNumber = 0;
					std::reverse(newSolution.begin(), newSolution.end());
					solution.clear();
					solution.insert(solution.begin(), newSolution.begin(), newSolution.end());
					assert(agent.isGoal(solution.back()->end, goal));
				}

				return solution;
			}

			bool addedToTree = treeInterface.insertIntoTree(e);

			if(!addedToTree) {
				pool.destroy(e);
			} else {
				poolEdges.emplace(e);
			}
		}

		if(!quiet) {
			dfpair(stdout, "solution cost", "-1");
			dfpair(stdout, "solution length", "-1");
		}
		return std::vector<const Edge*>();
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
	std::unordered_set<Edge*> poolEdges;
	std::vector<const Edge *> solution;
	std::vector<const Edge *> treeEdges;
	std::vector<State> samples;
	double solutionCost;
	double steeringDT, collisionCheckDT;
	int poseNumber;

	unsigned int samplesGenerated, edgesAdded, edgesRejected;

	bool quiet;
	double gBound;
};