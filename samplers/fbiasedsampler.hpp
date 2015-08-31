#pragma once

#include <random>

template <class Workspace, class Agent, class NN, class WorkspaceDiscretization>
class FBiasedSampler {
	typedef typename Agent::State State;
	typedef typename Agent::StateVars StateVars;
	typedef typename Agent::StateVarRanges StateVarRanges;

	struct Node {
		Node() : g(-1), h(-1) {}
		static double getG(const Node &n) {
			return n.g;
		}
		static void setG(Node &n, double c) {
			n.g = c;
		}
		static bool sortG(const Node *n1, const Node *n2) {
			return n1->g > n2->g;
		}

		static double getH(const Node &n) {
			return n.h;
		}
		static void setH(Node &n, double c) {
			n.h = c;
		}
		static bool sortH(const Node *n1, const Node *n2) {
			return n1->h > n2->h;
		}

		unsigned int id;
		double g, h, f;
		double score;
		double min, max;
	};

public:
	FBiasedSampler(const Workspace &workspace, const Agent &agent, NN &nn, const WorkspaceDiscretization &discretization,
	               const State &start, const State &goal, double omega = 4) :
		workspace(workspace), agent(agent), nn(nn), discretization(discretization), omega(omega) {

		unsigned int startIndex = discretization.getContainingCellId(start.getStateVars());
		unsigned int goalIndex = discretization.getContainingCellId(goal.getStateVars());
		unsigned int discretizationCells = discretization.getCellCount();

		nodes.resize(discretizationCells);

		dijkstraG(startIndex, nodes);
		dijkstraH(goalIndex, nodes);

		double minF = std::numeric_limits<double>::infinity();
		std::vector<Node *> untouched;
		for(Node &n : nodes) {
			n.f = n.g + n.h;
			if(n.f >= 0) {
				if(n.f < minF) {
					minF = n.f;
				}
			} else {
				untouched.push_back(&n);
			}
		}

		double numerator = pow(minF, omega);
		double minScore = std::numeric_limits<double>::infinity();
		double scoreSum = 0;
		for(Node &n : nodes) {
			if(n.f >= 0) {
				n.score = numerator / pow(n.f, omega);
				scoreSum += n.score;
				if(n.score <= minScore) {
					minScore = n.score;
				}
			}
		}

		minScore /= 2;
		for(Node *n : untouched) {
			n->score = minScore;
		}

		scoreSum += (double)untouched.size() * minScore;

		// unsigned int num = 0;
		double lowerBound = 0;
		for(Node &n : nodes) {
			n.min = lowerBound;
			n.max = lowerBound + n.score / scoreSum;
			assert(n.max - n.min > 0);
			lowerBound = n.max;
		}
	}

	State getTreeSample() const {
		auto sample = sampleConfiguration();
		auto sampleEdge = Edge(sample);
		typename NN::KNNResult result = nn.nearest(&sampleEdge);
		return result.elements[0]->end;
	}

private:

	State sampleConfiguration() const {
		StateVars vars;

		const Node &n = getNode(distribution(GlobalRandomGenerator));

		std::vector< std::vector<double> > cellBounds = discretization.getCellBoundingHyperRect(n.id);

		vars.resize(cellBounds.size());

		std::vector< std::uniform_real_distribution<double> > distributions;

		for(const std::vector<double> &cellBound : cellBounds) {
			distributions.emplace_back(cellBound[0], cellBound[1]);
		}

		while(true) {
			for(unsigned int i = 0; i < distributions.size(); ++i) {
				vars[i] = distributions[i](GlobalRandomGenerator);
			}

			if(discretization.getContainingCellId(vars) == n.id) {
				return agent.buildState(vars);
			}
		}
	};

	const Node &getNode(double value) const {
		unsigned int min = 0;
		unsigned int max = nodes.size();
		while(true) {
			unsigned int pivot = min + (max - min) / 2;
			const Node &n = nodes[pivot];
			if(value < n.min) {
				max = pivot - 1;
			} else if(value > n.max) {
				min = pivot + 1;
			} else {
				return n;
			}
		}
	}

	void dijkstraG(unsigned int start, std::vector<Node> &nodes) const {
		dijkstra(start, nodes, Node::getG, Node::setG, Node::sortG);
	}

	void dijkstraH(unsigned int start, std::vector<Node> &nodes) const {
		dijkstra(start, nodes, Node::getH, Node::setH, Node::sortH);
	}

	void dijkstra(unsigned int start, std::vector<Node> &nodes,
	              std::function<double(const Node &)> peek,
	              std::function<void(Node &, double)> update,
	              std::function<bool(const Node *, const Node *)> sort) const {

		std::vector<const Node *> queue(1, &nodes[start]);
		nodes[start].id = start;
		update(nodes[start], 0);

		while(!queue.empty()) {
			const Node *cur = queue.front();
			std::pop_heap(queue.begin(),queue.end(), sort);
			queue.pop_back();

			double curDist = peek(*cur);

			std::vector<unsigned int> neighbors = discretization.getNeighbors(cur->id);
			for(auto neighbor : neighbors) {
				Node &node = nodes[neighbor];
				if(peek(node) >= 0) continue; //duplicate
				node.id = neighbor;
				update(node, curDist + discretization.getCostBetweenCells(cur->id, neighbor));
				queue.push_back(&node);
				std::push_heap(queue.begin(),queue.end(), sort);
			}
		}
	}


	const Workspace &workspace;
	const Agent &agent;
	NN &nn;
	const WorkspaceDiscretization &discretization;
	StateVarRanges stateVarDomains;
	std::vector<Node> nodes;
	mutable std::uniform_real_distribution<double> distribution;
	double omega;
};