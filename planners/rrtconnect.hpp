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

		maxExtensions = args.doubleVal("RRTConnect Max Extensions");

		dfpair(stdout, "steering dt", "%g", steeringDT);
		dfpair(stdout, "collision check dt", "%g", collisionCheckDT);
	}


	void query(const State &start, const State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {
		bool foundGoal = false;
#ifdef WITHGRAPHICS
		auto green = OpenGLWrapper::Color::Green();
		start.draw(green);
		agent.drawMesh(start);
		goal.draw(green);
#endif

		if(agent.isGoal(start, goal)) {
			fprintf(stderr, "found goal\n");
			return;
		}

		if(firstInvocation) {
			auto root = pool.construct(start);
			treeInterface.insertIntoTree(root);
		}

		unsigned int iterations = 0;

		while(!foundGoal) {

			Edge *treeSample = treeInterface.getTreeSample();
			samplesGenerated++;

#ifdef WITHGRAPHICS
			samples.push_back(treeSample->end);
#endif

			auto edge = agent.randomSteer(treeSample->end, steeringDT);

			unsigned int added = 0;
			Edge *parent = treeSample;

			while(added < maxExtensions && workspace.safeEdge(agent, edge, collisionCheckDT)) {
				added++;
				edgesAdded++;
				Edge *e = pool.construct(edge);
				bool addedIntoTree = treeInterface.insertIntoTree(e);
				if(!addedIntoTree) {
					pool.destroy(e);
					break;
				}

				e->parent = parent;
				parent = e;

#ifdef WITHGRAPHICS
				treeEdges.push_back(e);
#endif

				if(agent.isGoal(edge.end, goal)) {
					fprintf(stderr, "found goal\n");
					std::vector<const Edge *> newSolution;
					double newSolutionCost = 0;
					newSolution.push_back(pool.construct(edge));
					newSolutionCost += edge.cost;
					while(newSolution.back()->parent != NULL) {
						newSolution.push_back(newSolution.back()->parent);
						newSolutionCost += newSolution.back()->cost;
					}
					if(solutionCost < 0 || newSolutionCost < solutionCost) {
						std::reverse(newSolution.begin(), newSolution.end());
						solution.clear();
						solution.insert(solution.begin(), newSolution.begin(), newSolution.end());
					}
					foundGoal = true;
					break;
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