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
		steeringDT = args.doubleVal("Steering Delta t");
		collisionCheckDT = args.doubleVal("Collision Check Delta t");

		maxExtensions = args.integerVal("RRTConnect Max Extensions");

		dfpair(stdout, "steering dt", "%g", steeringDT);
		dfpair(stdout, "collision check dt", "%g", collisionCheckDT);
		dfpair(stdout, "max extensions", "%u", maxExtensions);
	}


	std::vector<const Edge*> query(const State &start, const State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {
		bool foundGoal = false;
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

		while(!foundGoal) {

			std::pair<Edge*, State> treeSample = treeInterface.getTreeSample();
			samplesGenerated++;

#ifdef WITHGRAPHICS
			samples.push_back(treeSample.second);
#endif

			auto edge = agent.steer(treeSample.first->end, treeSample.second, steeringDT);

			unsigned int added = 0;
			Edge *parent = treeSample.first;

			while(added < maxExtensions && workspace.safeEdge(agent, edge, collisionCheckDT)) {
				added++;
				edgesAdded++;
				Edge *e = pool.construct(edge);
				bool addedIntoTree = treeInterface.insertIntoTree(e);
				if(!addedIntoTree) {
					pool.destroy(e);
					break;
				}

				e->updateParent(parent);
				parent = e;

#ifdef WITHGRAPHICS
				treeEdges.push_back(e);
#endif

				if(agent.isGoal(e->end, goal)) {
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
						std::reverse(newSolution.begin(), newSolution.end());
						solution.clear();
						solution.insert(solution.begin(), newSolution.begin(), newSolution.end());
					}
					foundGoal = true;

#ifdef WITHGRAPHICS
					break;
#endif
					return solution;
				}

				edge = agent.steerWithControl(edge.end, edge, steeringDT);
			}

			edgesRejected++;

			++iterations;
			if(iterationsAtATime > 0 && ++iterations > iterationsAtATime) break;
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
			agent.drawSolution(solution);
			// if(poseNumber >= solution.size() * 2) poseNumber = -1;
			// if(poseNumber >= 0)
			// 	agent.animateSolution(solution, poseNumber++);
			return solution;
		}
#endif

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
	unsigned int maxExtensions, samplesGenerated, edgesAdded, edgesRejected;
};