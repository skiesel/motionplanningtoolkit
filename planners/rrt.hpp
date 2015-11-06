#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>
#include "../utilities/datafile.hpp"

template<class Workspace, class Agent, class TreeInterface>
class RRT {
public:

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	RRT(const Workspace &workspace, const Agent &agent, TreeInterface &treeInterface, const InstanceFileMap &args, bool quiet = false,
		double gBound = std::numeric_limits<double>::infinity()) :
		workspace(workspace), agent(agent), treeInterface(treeInterface), solutionCost(-1), poseNumber(-1),
		samplesGenerated(0), edgesAdded(0), edgesRejected(0), quiet(quiet), gBound(gBound) {
		steeringDT = args.doubleVal("Steering Delta t");
		collisionCheckDT = args.doubleVal("Collision Check Delta t");

		if(!quiet) {
			dfpair(stdout, "steering dt", "%g", steeringDT);
			dfpair(stdout, "collision check dt", "%g", collisionCheckDT);
		}
	}


	std::vector<const Edge*> query(const State &start, const State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {

#ifdef WITHGRAPHICS
		auto green = OpenGLWrapper::Color::Green();
		start.draw(green);
		goal.draw(green);
#endif

		if(agent.isGoal(start, goal)) {
			if(!quiet) {
				dfpair(stdout, "solution cost", "0");
				dfpair(stdout, "solution length", "0");
			}
			return std::vector<const Edge*>();
		}

		if(firstInvocation) {
			auto root = pool.construct(start);
			treeInterface.insertIntoTree(root);
		}

		unsigned int iterations = 0;

		while(true) {

			std::pair<Edge*, State> treeSample = treeInterface.getTreeSample();
			samplesGenerated++;

#ifdef WITHGRAPHICS
			samples.push_back(treeSample.second);
#endif
			auto edge = agent.steer(treeSample.first->end, treeSample.second, steeringDT);

			edge.updateParent(treeSample.first);

			if(edge.gCost() >= gBound) {
				continue;
			}

			++iterations;

			if(!workspace.safeEdge(agent, edge, collisionCheckDT)) {
				edgesRejected++;

				if(iterationsAtATime > 0 && iterations >= iterationsAtATime) break;

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

#ifdef WITHGRAPHICS
				break;
#endif
				return solution;
			}

			bool addedToTree = treeInterface.insertIntoTree(e);

			if(!addedToTree) {
				pool.destroy(e);
			}

#ifdef WITHGRAPHICS
			if(addedToTree) {
				treeEdges.push_back(e);
			}
#endif

			if(iterationsAtATime > 0 && iterations >= iterationsAtATime) break;
		}

#ifdef WITHGRAPHICS
		for(const Edge *edge : treeEdges) {
			edge->draw(OpenGLWrapper::Color::Red());
		}

		treeEdges.clear();

		for(const State &sample : samples) {
			sample.draw();
		}

		samples.clear();

		if(solution.size() > 0) {
			auto red = OpenGLWrapper::Color::Red();
			for(const Edge *edge : solution) {
				edge->draw(red);
			}
		// 	agent.drawSolution(solution, collisionCheckDT);
		// 	// if(poseNumber >= solution.size() * 2) poseNumber = -1;
		// 	// if(poseNumber >= 0)
		// 	// 	agent.animateSolution(solution, poseNumber++);
			assert(agent.isGoal(solution.back()->end, goal));
			return solution;
		}
#endif

#ifdef VREPPLUGIN
		// if(solution.size() > 0) {
		// 	if(agent.validateSolution(solution, goal)) {
		// 		fprintf(stderr, "VALID SOLUTION!\n");
		// 	} else {
		// 		fprintf(stderr, "INVALID SOLUTION!\n");
		// 	}
		// 	agent.animateSolution(solution);
		// }
#endif

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