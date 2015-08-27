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


	void query(const State &start, const State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {

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

		while(true) {

			State treeSample = treeInterface.getTreeSample();
			samplesGenerated++;

#ifdef WITHGRAPHICS
			// treeSample.print();
			samples.push_back(treeSample);
#endif

			auto edge = agent.randomSteer(treeSample, steeringDT);

			++iterations;

			if(!workspace.safeEdge(agent, edge, collisionCheckDT)) {
				edgesRejected++;

				if(iterationsAtATime > 0 && iterations >= iterationsAtATime) break;

				continue;
			}

			edgesAdded++;

			if(agent.isGoal(edge.end, goal)) {
				fprintf(stderr, "found goal\n");
				std::vector<const Edge *> newSolution;
				double newSolutionCost = 0;
				State cur = edge.start;
				newSolution.push_back(pool.construct(edge));
				newSolutionCost += edge.cost;
				while(!cur.equals(start)) {
					auto e = treeInterface.getTreeEdge(cur);
					newSolution.push_back(e);
					newSolutionCost += e->cost;
					cur = e->start;
				}
				if(solutionCost < 0 || newSolutionCost < solutionCost) {
					poseNumber = 0;
					std::reverse(newSolution.begin(), newSolution.end());
					solution.clear();
					solution.insert(solution.begin(), newSolution.begin(), newSolution.end());
				}

				break;
			}

			Edge *e = pool.construct(edge);

			treeInterface.insertIntoTree(e);

#ifdef WITHGRAPHICS
			treeEdges.push_back(e);
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