#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>
#include "../utilities/datafile.hpp"

template<class Workspace, class Agent, class TreeInterface>
class RRT {
public:

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	RRT(const Workspace &workspace, const Agent &agent, TreeInterface &treeInterface, const InstanceFileMap &args) :
		workspace(workspace), agent(agent), treeInterface(treeInterface), solutionCost(-1), poseNumber(-1),
		samplesGenerated(0), edgesAdded(0), edgesRejected(0) {
		steeringDT = args.doubleVal("Steering Delta t");
		collisionCheckDT = args.doubleVal("Collision Check Delta t");

		dfpair(stdout, "steering dt", "%g", steeringDT);
		dfpair(stdout, "collision check dt", "%g", collisionCheckDT);
	}


	std::vector<const Edge*> query(const State &start, const State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {

#ifdef WITHGRAPHICS
		auto green = OpenGLWrapper::Color::Green();
		start.draw(green);
		agent.drawMesh(start);
		goal.draw(green);
#endif

		if(agent.isGoal(start, goal)) {
			dfpair(stdout, "solution cost", "0");
			dfpair(stdout, "solution length", "0");
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

			++iterations;

			if(!workspace.safeEdge(agent, edge, collisionCheckDT)) {
				edgesRejected++;

				if(iterationsAtATime > 0 && iterations >= iterationsAtATime) break;

				continue;
			}

			edgesAdded++;

			Edge *e = pool.construct(edge);
<<<<<<< HEAD
			e->updateParent(treeSample);

			if(agent.isGoal(e->end, goal)) {

=======
			e->updateParent(treeSample.first);

			if(agent.isGoal(e->end, goal)) {

>>>>>>> skiesel/master
				dfpair(stdout, "solution cost", "%g", e->gCost());
				std::vector<const Edge *> newSolution;
				newSolution.push_back(e);

				unsigned int edgeCount = 1;
				while(newSolution.back()->parent != NULL) {
					edgeCount++;
					newSolution.push_back(newSolution.back()->parent);
				}
				dfpair(stdout, "solution length", "%u", edgeCount);
				if(solutionCost < 0 || e->gCost() < solutionCost) {
					poseNumber = 0;
					std::reverse(newSolution.begin(), newSolution.end());
					solution.clear();
					solution.insert(solution.begin(), newSolution.begin(), newSolution.end());
				}

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

		for(const State &sample : samples) {
			sample.draw();
		}

		if(solution.size() > 0) {
			auto red = OpenGLWrapper::Color::Red();
			for(const Edge *edge : solution) {
				edge->draw(red);
			}
			agent.drawSolution(solution, collisionCheckDT);
			// if(poseNumber >= solution.size() * 2) poseNumber = -1;
			// if(poseNumber >= 0)
			// 	agent.animateSolution(solution, poseNumber++);
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

		dfpair(stdout, "solution cost", "-1");
		dfpair(stdout, "solution length", "-1");
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
};