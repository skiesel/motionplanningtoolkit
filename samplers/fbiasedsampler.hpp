#pragma once

#include <random>

template <class Workspace, class Agent, class WorkspaceDiscretization>
class FBiasedSampler {
	typedef typename Agent::State State;
	typedef typename Agent::StateVars StateVars;
	typedef typename Agent::StateVarRanges StateVarRanges;

	struct Node {
		Node() : g(-1), h(-1) {}
		static double getG(const Node &n) { return n.g; }
		static void setG(Node &n, double c) { n.g = c; }
		static bool sortG(const Node *n1, const Node *n2) { return n1->g > n2->g; }

		static double getH(const Node &n) { return n.h; }
		static void setH(Node &n, double c) { n.h = c; }
		static bool sortH(const Node *n1, const Node *n2) { return n1->h > n2->h; }

		unsigned int id;
		double g, h;
	};

public:
	FBiasedSampler(const Workspace &workspace, const Agent &agent, const WorkspaceDiscretization& discretization,
				const State& start, const State& goal) :
		workspace(workspace), agent(agent), discretization(discretization) {

			unsigned int startIndex = discretization.getContainingCellId(start.getStateVars());
			unsigned int goalIndex = discretization.getContainingCellId(goal.getStateVars());
			unsigned int discretizationCells = discretization.getCellCount();

			std::vector<Node> nodes(discretizationCells);

			dijkstraG(startIndex, nodes);
			dijkstraH(goalIndex, nodes);
		}

	State sampleConfiguration() const {
		StateVars vars;
		return agent.buildState(vars);
	};
private:
	void dijkstraG(unsigned int start, std::vector<Node>& nodes) const {
		dijkstra(start, nodes, Node::getG, Node::setG, Node::sortG);
	}

	void dijkstraH(unsigned int start, std::vector<Node>& nodes) const {
		dijkstra(start, nodes, Node::getH, Node::setH, Node::sortH);
	}

	void dijkstra(unsigned int start, std::vector<Node>& nodes,
					std::function<double(const Node&)> peek,
					std::function<void(Node&, double)> update,
					std::function<bool(const Node*, const Node*)> sort) const {
		
		std::vector<const Node*> queue(1, &nodes[start]);
		nodes[start].id = start;
		update(nodes[start], 0);

		while(!queue.empty()) {
			const Node *cur = queue.front(); std::pop_heap(queue.begin(),queue.end(), sort); queue.pop_back();

			double curDist = peek(*cur);

			std::vector<unsigned int> neighbors = discretization.getNeighbors(cur->id);
			for(auto neighbor : neighbors) {
				Node &node = nodes[neighbor];
				if(peek(node) >= 0) continue; //duplicate
				node.id = neighbor;
				update(node, curDist + discretization.getCostBetweenCells(cur->id, neighbor));
				queue.push_back(&node); std::push_heap(queue.begin(),queue.end(), sort);
			}
		}
	}


	const Workspace &workspace;
	const Agent &agent;
	const WorkspaceDiscretization &discretization;
	StateVarRanges stateVarDomains;

};