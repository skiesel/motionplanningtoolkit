#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>

template<class Workspace, class Agent, class Sampler, class NN>
class RRT {
public:

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	RRT(const Workspace &workspace, const Agent &agent, const Sampler &sampler, NN &nn, const InstanceFileMap &args) :
		workspace(workspace), agent(agent), sampler(sampler), nn(nn), solutionCost(-1) {
			steeringDT = stod(args.value("Steering Delta t"));
			collisionCheckDT = stod(args.value("Collision Check Delta t"));
		}


	void query(const State &start, const State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {

#ifdef WITHGRAPHICS
		auto green = OpenGLWrapper::Color::Green();
		start.draw(green);
		goal.draw(green);
#endif

		if(firstInvocation) {
			if(agent.isGoal(start, goal)) {
				return;
			}

			auto root = pool.construct(start, start, 0);

			nn.insertPoint(root);
		}

		unsigned int iterations = 0;

		while(true) {
			auto sample = sampler.sampleConfiguration();

#ifdef WITHGRAPHICS
			samples.push_back(sample);
#endif

			//intentionally not in the pool
			auto sampleEdge = Edge(sample, sample, 0);

			typename NN::KNNResult result = nn.nearest(&sampleEdge);

			State nearest = result.elements[0]->end;

			auto edge = agent.steer(nearest, sample, steeringDT);
			//auto edge = agent.randomSteer(nearest, steeringDT);

			if(!workspace.safeEdge(agent, edge, collisionCheckDT)) {
				++iterations
				continue;
			}

			if(agent.isGoal(edge.end, goal)) {
				std::vector<const Edge*> newSolution;
				double newSolutionCost = 0;
				State cur = edge.start;
				newSolution.push_back(pool.construct(edge.start, edge.end, edge.cost));
				newSolutionCost += edge.cost;
				while(!cur.equals(start)) {
					auto e = Edge(cur, cur, 0);
					typename NN::KNNResult r = nn.nearest(&e);
					newSolution.push_back(r.elements[0]);
					newSolutionCost += r.elements[0]->cost;
					cur = r.elements[0]->start;
				}
				if(solutionCost < 0 || newSolutionCost < solutionCost) {
					std::reverse(newSolution.begin(), newSolution.end());
					solution.clear();
					solution.insert(solution.begin(), newSolution.begin(), newSolution.end());
				}
				
				break;
			}

			Edge *e = pool.construct(edge.start, edge.end, edge.cost);
			nn.insertPoint(e);

#ifdef WITHGRAPHICS
			treeEdges.push_back(e);
#endif

			if(iterationsAtATime > 0 && ++iterations > iterationsAtATime) break;
		}

#ifdef WITHGRAPHICS
		for(const Edge* edge : treeEdges) {
			edge->draw();
		}

		for(const State &sample : samples) {
			sample.draw();
		}

		auto red = OpenGLWrapper::Color::Red();
		for(const Edge *edge : solution) {
			edge->draw(red);
		}
#endif
	}
private:
	const Workspace &workspace;
	const Agent &agent;
	const Sampler &sampler;
	NN &nn;
	boost::object_pool<Edge> pool;
	std::vector<const Edge*> solution;
	std::vector<const Edge*> treeEdges;
	std::vector<State> samples;
	double solutionCost;
	double steeringDT, collisionCheckDT;
};