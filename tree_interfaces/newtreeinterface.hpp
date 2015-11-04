#pragma once

#include <random>

template <class Workspace, class Agent, class NN, class WorkspaceDiscretization>
class NewTreeInterface {
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;
	typedef typename Agent::StateVars StateVars;
	typedef typename Agent::StateVarRanges StateVarRanges;

	struct Node {
		Node() : g(std::numeric_limits<double>::infinity()), h(std::numeric_limits<double>::infinity()), e(std::numeric_limits<double>::infinity()) {}
		static double getG(const Node &n) {
			return n.g;
		}
		static void setG(Node &n, double c) {
			n.g = c;
		}
		static bool sortG(const Node *n1, const Node *n2) {
			return n1->g < n2->g;
		}

		static double getH(const Node &n) {
			return n.h;
		}
		static void setH(Node &n, double c) {
			n.h = c;
		}
		static bool sortH(const Node *n1, const Node *n2) {
			return n1->h < n2->h;
		}

		static double getE(const Node &n) {
			return n.e;
		}
		static void setE(Node &n, double c) {
			n.e = c;
		}
		static bool sortE(const Node *n1, const Node *n2) {
			return n1->e < n2->e;
		}

		unsigned int heapIndex;
		static bool pred(const Node *a, const Node *b) {
			return sort(a,b);
		}
		static unsigned int getHeapIndex(const Node *r) {
			return r->heapIndex;
		}
		static void setHeapIndex(Node *r, unsigned int i) {
			r->heapIndex = i;
		}


		static std::function<bool(const Node *, const Node *)> sort;
		unsigned int id, nextNode;
		double g, h, e ;

		FrequencyTreeInterface<Agent> regionManager;
	};

public:
	NewTreeInterface(const Workspace &workspace, const Agent &agent, NN &nn, const WorkspaceDiscretization &discretization,
	               const State &start, const State &goal, double stateRadius, double goalBias) :
		workspace(workspace), agent(agent), nn(nn), discretization(discretization), goal(goal), stateRadius(stateRadius), goalBias(goalBias) {

		unsigned int startIndex = discretization.getCellId(start);
		unsigned int goalIndex = discretization.getCellId(goal);
		unsigned int discretizationCells = discretization.getCellCount();

		nodes.resize(discretizationCells);

		for(unsigned int i = 0; i < discretizationCells; ++i) {
			nodes[i].id = i;
		}

		// dijkstraG(nodes[startIndex]);
		// dijkstraH(nodes[goalIndex]);
		dijkstraE(nodes[goalIndex]);

		open.push(&nodes[startIndex]);

		mostRecentNode = NULL;
	}

	std::pair<Edge*, State> getTreeSample() {
		if(mostRecentNode != NULL) {
			//there was a collision so increase effort
			increaseEffort(mostRecentNode->id, 1);
			open.siftFromItem(&nodes[mostRecentNode->id]);
			mostRecentNode = NULL;
		}

		if(distribution(GlobalRandomGenerator) < goalBias) {
			auto edge = Edge(goal);
			typename NN::KNNResult result = nn.nearest(&edge);
			return std::make_pair(result.elements[0], goal);
		} else {
			mostRecentNode = open.peek();
			State sample = discretization.getRandomStateNearRegionCenter(nodes[mostRecentNode->nextNode].id, stateRadius);

			//automatically updates chosen count
			Edge *edge = mostRecentNode->regionManager.getTreeSample().first;
			return std::make_pair(edge, sample);
		}
	}

	Edge *getTreeEdge(const State &s) const {
		auto edge = Edge(s);
		typename NN::KNNResult result = nn.nearest(&edge);
		return result.elements[0];
	}

	bool insertIntoTree(Edge *edge) {
		unsigned int index = discretization.getCellId(edge->end);

		if(mostRecentNode != NULL) {
			double amount = mostRecentNode->id == index ? 0.5 : 0.1;
			increaseEffort(mostRecentNode->id, amount);
			open.siftFromItem(&nodes[mostRecentNode->id]);
			mostRecentNode = NULL;
		}

		nn.insertPoint(edge);
		nodes[index].regionManager.insertIntoTree(edge);

		if(!open.inHeap(&nodes[index])) {
			open.push(&nodes[index]);
		}
		
		return true;
	}

	void removeFromTree(Edge *edge) {
		//remove
	}

#ifdef WITHGRAPHICS
	void draw() const {}
#endif

private:

	void increaseEffort(unsigned int id, double howMuch) {
		nodes[id].e += howMuch;

		if(open.inHeap(&nodes[id])) {
			open.siftFromItem(&nodes[id]);
		}

		std::vector<unsigned int> updates;
		unsigned int index = 0;
		updates.push_back(id);

		while(index < updates.size()) {
			unsigned int current = updates[index];

			auto neighbors = discretization.getNeighboringCells(current);
			for(auto neighbor : neighbors) {
				if(nodes[neighbor].nextNode == id) {
					double oldValue = nodes[neighbor].e;
					nodes[neighbor].e += howMuch;

					auto neighbors2 = discretization.getNeighboringCells(neighbor);
					for(auto neighbor2 : neighbors2) {
						if(nodes[neighbor2].e + 1 < nodes[neighbor].e) {
							nodes[neighbor].e = nodes[neighbor2].e + 1;
							nodes[neighbor].nextNode = neighbor2;
						}
					}
					if(nodes[neighbor].e > oldValue) {
						if(open.inHeap(&nodes[neighbor])) {
							open.siftFromItem(&nodes[neighbor]);
						}
						updates.push_back(neighbor);
					}
					break;
				}
			}
			index++;
		}
	}

	// void dijkstraG(Node &start) {
	// 	Node::sort = Node::sortG;
	// 	dijkstra(start, Node::getG, Node::setG);
	// }

	// void dijkstraH(Node &start) {
	// 	Node::sort = Node::sortH;
	// 	dijkstra(start, Node::getH, Node::setH);
	// }

	void dijkstraE(Node &start) {
		Node::sort = Node::sortE;
		dijkstra(start, Node::getE, Node::setE);
	}

	void dijkstra(Node &start,
					std::function<double(const Node &)> peek,
	              	std::function<void(Node &, double)> update) {

		InPlaceBinaryHeap<Node, Node> open;
		std::unordered_set<unsigned int> closed;
		update(start, 0);
		open.push(&start);

		closed.insert(start.id);

		while(!open.isEmpty()) {
			Node *current = open.pop();

			closed.insert(current->id);

			std::vector<unsigned int> kids = discretization.getNeighboringCells(current->id);
			for(unsigned int kid : kids) {
				if(closed.find(kid) != closed.end()) continue;

				double newValue = peek(*current) + 1;
				Node &kidRef = nodes[kid];

				if(newValue < peek(kidRef)) {
					update(kidRef, newValue);
					kidRef.nextNode = current->id;

					if(open.inHeap(&kidRef)) {
						open.siftFromItem(&kidRef);
					} else {
						open.push(&kidRef);
					}
				}
			}
		}
	}

	const Workspace &workspace;
	const Agent &agent;
	NN &nn;
	const WorkspaceDiscretization &discretization;
	const State &goal;
	std::vector<Node> nodes;
	InPlaceBinaryHeap<Node, Node> open;
	double stateRadius, goalBias;
	Node *mostRecentNode;
	mutable std::uniform_real_distribution<double> distribution;
};

template <class W, class A, class N, class D>
std::function<bool(const typename NewTreeInterface<W,A,N,D>::Node *, const typename NewTreeInterface<W,A,N,D>::Node *)> NewTreeInterface<W,A,N,D>::Node::sort = NewTreeInterface<W,A,N,D>::Node::sortG;


