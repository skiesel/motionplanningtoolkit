#pragma once

#include <random>

template <class Workspace, class Agent, class NN, class WorkspaceDiscretization>
class FBiasedSampler {
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;
	typedef typename Agent::StateVars StateVars;
	typedef typename Agent::StateVarRanges StateVarRanges;

	struct Node {
		Node() : g(std::numeric_limits<double>::infinity()), h(std::numeric_limits<double>::infinity()), inPDF(false), touched(false), pdfID(0) {}
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
		unsigned int id;
		double g, h, f;
		double score;
		bool inPDF, touched;
		unsigned int pdfID;
	};

public:
	FBiasedSampler(const Workspace &workspace, const Agent &agent, NN &nn, const WorkspaceDiscretization &discretization,
	               const State &start, const State &goal, double stateRadius, double omega = 4, bool addAllRegions = true, double shellPreference = 1) :
		workspace(workspace), agent(agent), nn(nn), discretization(discretization), omega(omega), stateRadius(stateRadius), shellPreference(shellPreference) {

		unsigned int startIndex = discretization.getCellId(start);
		unsigned int goalIndex = discretization.getCellId(goal);
		unsigned int discretizationCells = discretization.getCellCount();

		nodes.resize(discretizationCells);

		for(unsigned int i = 0; i < discretizationCells; ++i) {
			nodes[i].id = i;
		}

		dijkstraG(nodes[startIndex]);
		dijkstraH(nodes[goalIndex]);

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

		if(addAllRegions) {
			for(unsigned int i = 0; i < nodes.size(); ++i) {
				auto el = pdf.add(&nodes[i], nodes[i].score);
				nodes[i].pdfID = el->getId();
				nodes[i].inPDF = true;
			}
		} else {
			auto el = pdf.add(&nodes[startIndex], nodes[startIndex].score);
			nodes[startIndex].pdfID = el->getId();
			nodes[startIndex].inPDF = true;
			Edge * e = new Edge(start);
			reached(e, 0.5); //this should be passed in...
			delete e;
		}
	}

	std::pair<Edge*, State> getTreeSample() const {
		Node *randomNode = pdf.sample();
		State sample = discretization.getRandomStateNearRegionCenter(randomNode->id, stateRadius);
		Edge sampleEdge = Edge(sample);
		typename NN::KNNResult result = nn.nearest(&sampleEdge);
		return std::make_pair(result.elements[0], sample);
	}

	Edge *getTreeEdge(const State &s) const {
		auto edge = Edge(s);
		typename NN::KNNResult result = nn.nearest(&edge);
		return result.elements[0];
	}

	void reached(const Edge *e, double shellDepth) {

		struct node {
			node(unsigned int id, unsigned int depth, double value) : id(id), value(value) {}

			static bool pred(const node *a, const node *b) {
				return a->value < b->value;
			}
			static unsigned int getHeapIndex(const node *r) {
				return r->heapIndex;
			}
			static void setHeapIndex(node *r, unsigned int i) {
				r->heapIndex = i;
			}

			unsigned int id, depth, heapIndex;
			double value;
		};

		InPlaceBinaryHeap<node, node> open;
		std::unordered_set<unsigned int> closed;
		std::unordered_map<unsigned int, node*> lookup;

		unsigned int start = discretization.getCellId(e->end);
		lookup[start] = new node(start, 0, 0);
		if(!nodes[start].touched) {
			if(nodes[start].inPDF) {
				nodes[start].score /= shellPreference;
				pdf.update(nodes[start].pdfID, nodes[start].score);
			} else {
				auto el = pdf.add(&nodes[start], nodes[start].score);
				nodes[start].pdfID = el->getId();
				nodes[start].inPDF = true;
			}
			nodes[start].touched = true;
		}

		open.push(lookup[start]);
		while(!open.isEmpty()) {
			node *current = open.pop();
			closed.insert(current->id);

			std::vector<unsigned int> kids = discretization.getNeighboringCells(current->id);
			for(unsigned int kid : kids) {
				if(closed.find(kid) != closed.end()) continue;

				double newValue = current->value + discretization.getEdgeCostBetweenCells(current->id, kid);
				if(newValue <= shellDepth /*|| current->depth == 0*/) {
					if(lookup.find(kid) != lookup.end()) {
						if(newValue < lookup[kid]->value) {
							lookup[kid]->value = newValue;
							lookup[kid]->depth = current->depth + 1;
							open.siftFromItem(lookup[kid]);
						}
					} else {
						lookup[kid] = new node(kid, current->depth + 1, newValue);
						open.push(lookup[kid]);
					}
				}
			}
		}

		for(auto entry : lookup) {
			auto n = entry.first;
			delete entry.second;
			if(!nodes[n].inPDF) {
				nodes[n].score *= shellPreference;
				auto el = pdf.add(&nodes[n], nodes[n].score);
				nodes[n].pdfID = el->getId();
				nodes[n].inPDF = true;
			}
		}
	}

	void removed(const Edge *e) {
		fprintf(stderr, "FBiasedSampler removed() not implemented\n");
		exit(1);
	}

#ifdef WITHGRAPHICS
	void draw() const {
		std::vector<std::vector<double>> colorLookup(nodes.size());

		double min = 1000000000;
		double max = 0;
		for(unsigned int i = 0; i < nodes.size(); ++i) {
			min = std::min(nodes[i].score, min);
			if(!isinf(nodes[i].f))
				max = std::max(nodes[i].score, max);
		}

		for(unsigned int i = 0; i < nodes.size(); ++i) {
			colorLookup[i] = OpenGLWrapper::getColor(min, max, nodes[i].score);
		}

		discretization.draw(true, false, colorLookup);
	}
#endif

private:

	void dfsToDepth(unsigned int root, double value, std::unordered_map<unsigned int, double> &nodesInsideBound, double bound) const {
		nodesInsideBound[root] = value;
		std::vector<unsigned int> kids = discretization.getNeighboringCells(root);
		for(auto kid : kids) {
			double cost = value + discretization.getEdgeCostBetweenCells(root, kid);

			if(nodesInsideBound.find(kid) != nodesInsideBound.end()) {
				if(nodesInsideBound[kid] <= cost) {
					continue;
				}
			}

			if(cost < bound) {
				dfsToDepth(kid, cost, nodesInsideBound, bound);
			}
		}
	}

	void dijkstraG(Node &start) {
		Node::sort = Node::sortG;
		dijkstra(start, Node::getG, Node::setG);
	}

	void dijkstraH(Node &start) {
		Node::sort = Node::sortH;
		dijkstra(start, Node::getH, Node::setH);
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

				double newValue = peek(*current) + discretization.getEdgeCostBetweenCells(current->id, kid);
				Node &kidRef = nodes[kid];

				if(newValue < peek(kidRef)) {
					update(kidRef, newValue);

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
	std::vector<Node> nodes;
	ProbabilityDensityFunction<Node> pdf;
	double omega, stateRadius, shellPreference;
};

template <class W, class A, class N, class D>
std::function<bool(const typename FBiasedSampler<W,A,N,D>::Node *, const typename FBiasedSampler<W,A,N,D>::Node *)> FBiasedSampler<W,A,N,D>::Node::sort = FBiasedSampler<W,A,N,D>::Node::sortG;

// template <class Workspace, class Agent, class NN, class WorkspaceDiscretization>
// class FBiasedSampler {
// 	typedef typename Agent::State State;
// 	typedef typename Agent::Edge Edge;
// 	typedef typename Agent::StateVars StateVars;
// 	typedef typename Agent::StateVarRanges StateVarRanges;

// 	struct Node {
// 		Node() : g(-1), h(-1) {}
// 		static double getG(const Node &n) {
// 			return n.g;
// 		}
// 		static void setG(Node &n, double c) {
// 			n.g = c;
// 		}
// 		static bool sortG(const Node *n1, const Node *n2) {
// 			return n1->g > n2->g;
// 		}

// 		static double getH(const Node &n) {
// 			return n.h;
// 		}
// 		static void setH(Node &n, double c) {
// 			n.h = c;
// 		}
// 		static bool sortH(const Node *n1, const Node *n2) {
// 			return n1->h > n2->h;
// 		}

// 		unsigned int id;
// 		double g, h, f;
// 		double score;
// 		double min, max;
// 	};

// public:
// 	FBiasedSampler(const Workspace &workspace, const Agent &agent, NN &nn, const WorkspaceDiscretization &discretization,
// 	               const State &start, const State &goal, double omega = 4) :
// 		workspace(workspace), agent(agent), nn(nn), discretization(discretization), omega(omega) {

// 		unsigned int startIndex = discretization.getContainingCellId(start.getStateVars());
// 		unsigned int goalIndex = discretization.getContainingCellId(goal.getStateVars());
// 		unsigned int discretizationCells = discretization.getCellCount();

// 		nodes.resize(discretizationCells);

// 		dijkstraG(startIndex, nodes);
// 		dijkstraH(goalIndex, nodes);

// 		double minF = std::numeric_limits<double>::infinity();
// 		std::vector<Node *> untouched;
// 		for(Node &n : nodes) {
// 			n.f = n.g + n.h;
// 			if(n.f >= 0) {
// 				if(n.f < minF) {
// 					minF = n.f;
// 				}
// 			} else {
// 				untouched.push_back(&n);
// 			}
// 		}

// 		double numerator = pow(minF, omega);
// 		double minScore = std::numeric_limits<double>::infinity();
// 		double scoreSum = 0;
// 		for(Node &n : nodes) {
// 			if(n.f >= 0) {
// 				n.score = numerator / pow(n.f, omega);
// 				scoreSum += n.score;
// 				if(n.score <= minScore) {
// 					minScore = n.score;
// 				}
// 			}
// 		}

// 		minScore /= 2;
// 		for(Node *n : untouched) {
// 			n->score = minScore;
// 		}

// 		scoreSum += (double)untouched.size() * minScore;

// 		// unsigned int num = 0;
// 		double lowerBound = 0;
// 		for(Node &n : nodes) {
// 			n.min = lowerBound;
// 			n.max = lowerBound + n.score / scoreSum;
// 			assert(n.max - n.min > 0);
// 			lowerBound = n.max;
// 		}
// 	}

// 	std::pair<Edge*, State> getTreeSample() const {
// 		State sample = sampleConfiguration();
// 		Edge sampleEdge = Edge(sample);
// 		typename NN::KNNResult result = nn.nearest(&sampleEdge);
// 		return std::make_pair(result.elements[0], sample);
// 	}

// private:

// 	State sampleConfiguration() const {
// 		StateVars vars;

// 		const Node &n = getNode(distribution(GlobalRandomGenerator));

// 		std::vector< std::vector<double> > cellBounds = discretization.getCellBoundingHyperRect(n.id);

// 		vars.resize(cellBounds.size());

// 		std::vector< std::uniform_real_distribution<double> > distributions;

// 		for(const std::vector<double> &cellBound : cellBounds) {
// 			distributions.emplace_back(cellBound[0], cellBound[1]);
// 		}

// 		while(true) {
// 			for(unsigned int i = 0; i < distributions.size(); ++i) {
// 				vars[i] = distributions[i](GlobalRandomGenerator);
// 			}

// 			if(discretization.getContainingCellId(vars) == n.id) {
// 				return agent.buildState(vars);
// 			}
// 		}
// 	};

// 	const Node &getNode(double value) const {
// 		unsigned int min = 0;
// 		unsigned int max = nodes.size();
// 		while(true) {
// 			unsigned int pivot = min + (max - min) / 2;
// 			const Node &n = nodes[pivot];
// 			if(value < n.min) {
// 				max = pivot - 1;
// 			} else if(value > n.max) {
// 				min = pivot + 1;
// 			} else {
// 				return n;
// 			}
// 		}
// 	}

// 	void dijkstraG(unsigned int start, std::vector<Node> &nodes) const {
// 		dijkstra(start, nodes, Node::getG, Node::setG, Node::sortG);
// 	}

// 	void dijkstraH(unsigned int start, std::vector<Node> &nodes) const {
// 		dijkstra(start, nodes, Node::getH, Node::setH, Node::sortH);
// 	}

// 	void dijkstra(unsigned int start, std::vector<Node> &nodes,
// 	              std::function<double(const Node &)> peek,
// 	              std::function<void(Node &, double)> update,
// 	              std::function<bool(const Node *, const Node *)> sort) const {

// 		std::vector<const Node *> queue(1, &nodes[start]);
// 		nodes[start].id = start;
// 		update(nodes[start], 0);

// 		while(!queue.empty()) {
// 			const Node *cur = queue.front();
// 			std::pop_heap(queue.begin(),queue.end(), sort);
// 			queue.pop_back();

// 			double curDist = peek(*cur);

// 			std::vector<unsigned int> neighbors = discretization.getNeighbors(cur->id);
// 			for(auto neighbor : neighbors) {
// 				Node &node = nodes[neighbor];
// 				if(peek(node) >= 0) continue; //duplicate
// 				node.id = neighbor;
// 				update(node, curDist + discretization.getCostBetweenCells(cur->id, neighbor));
// 				queue.push_back(&node);
// 				std::push_heap(queue.begin(),queue.end(), sort);
// 			}
// 		}
// 	}

// 	const Workspace &workspace;
// 	const Agent &agent;
// 	NN &nn;
// 	const WorkspaceDiscretization &discretization;
// 	StateVarRanges stateVarDomains;
// 	std::vector<Node> nodes;
// 	mutable std::uniform_real_distribution<double> distribution;
// 	double omega;
// };