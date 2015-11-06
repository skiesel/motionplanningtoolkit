#pragma once

#include <random>

template <class Workspace, class Agent, class NN, class WorkspaceDiscretization>
class FBiasedShellSampler {
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;
	typedef typename Agent::StateVars StateVars;
	typedef typename Agent::StateVarRanges StateVarRanges;

	struct Node {
		Node() : g(std::numeric_limits<double>::infinity()), h(std::numeric_limits<double>::infinity()), inExteriorPDF(false), touched(false), pdfID(0) {}
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
		bool inExteriorPDF, touched;
		unsigned int pdfID;
	};

public:
	FBiasedShellSampler(const Workspace &workspace, const Agent &agent, NN &nn, const WorkspaceDiscretization &discretization,
	               const State &start, const State &goal, double stateRadius, double shellDepth, double omega = 4, double shellPreference = 1) :
		workspace(workspace), agent(agent), nn(nn), discretization(discretization), omega(omega), stateRadius(stateRadius), shellPreference(shellPreference), shellDepth(shellDepth) {

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

		Edge *e = new Edge(start);
		reached(e, shellDepth);
		delete e;
	}

	std::pair<Edge*, State> getTreeSample() const {
		Node *randomNode;
		if(zeroToOne(GlobalRandomGenerator) < shellPreference) {
			randomNode = exterior.sample();
		} else {
			randomNode = interior.sample();
		}

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
		//we have a local shellDepth passed in, and a object level shellDepth -- maybe we want to alter it during runtime?
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

		unsigned int start = discretization.getCellId(e->end);
		if(nodes[start].touched) {
			return;
		}

		InPlaceBinaryHeap<node, node> open;
		std::unordered_set<unsigned int> closed;
		std::unordered_map<unsigned int, node*> lookup;

		
		lookup[start] = new node(start, 0, 0);
		
		if(nodes[start].inExteriorPDF) {
			auto el = exterior.remove(nodes[start].pdfID);
			if(el != NULL) {
				el->getData()->pdfID = el->getId();
			}
		}

		auto el = interior.add(&nodes[start], nodes[start].score);
		nodes[start].pdfID = el->getId();
		nodes[start].inExteriorPDF = false;
		nodes[start].touched = true;

#ifdef WITHGRAPHICS		
		includeThese.insert(start);
#endif

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
			if(!(nodes[n].inExteriorPDF || nodes[n].touched)) {
				auto el = exterior.add(&nodes[n], nodes[n].score);
				nodes[n].pdfID = el->getId();
				nodes[n].inExteriorPDF = true;
#ifdef WITHGRAPHICS
				includeThese.insert(n);
#endif
			}
		}
	}

	void removed(const Edge *e) {
		fprintf(stderr, "FBiasedShellSampler removed() not implemented\n");
		exit(1);
	}
#ifdef WITHGRAPHICS
std::unordered_set<unsigned int> includeThese;

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

		discretization.draw(true, false, colorLookup, &includeThese);
	}
#endif

private:
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
	ProbabilityDensityFunction<Node> interior, exterior;
	double omega, stateRadius, shellPreference, shellDepth;
	mutable std::uniform_real_distribution<double> zeroToOne;
};

template <class W, class A, class N, class D>
std::function<bool(const typename FBiasedShellSampler<W,A,N,D>::Node *, const typename FBiasedShellSampler<W,A,N,D>::Node *)> FBiasedShellSampler<W,A,N,D>::Node::sort = FBiasedShellSampler<W,A,N,D>::Node::sortG;