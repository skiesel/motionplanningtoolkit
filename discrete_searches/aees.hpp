#pragma once

#include "../utilities/heap.hpp"
#include "../utilities/rbtree.hpp"

class AEES {

	struct Node {
		Node(unsigned int id, double g, double ghat, double h, double hhat, double f, double fhat, double d, double dhat, double searchEffort) :
			g(g), ghat(ghat), h(h), hhat(hhat), f(f), fhat(fhat), d(d), dhat(dhat), searchEffort(searchEffort), id(id),
			fIndex(std::numeric_limits<unsigned int>::max()), searchEffortIndex(std::numeric_limits<unsigned int>::max()) {}

		double g, ghat, h, hhat, f, fhat, d, dhat, searchEffort;
		unsigned int id, fIndex, searchEffortIndex;
	};

	struct SortOnF {
		static unsigned int getHeapIndex(const Node *n) {
			return n->fIndex;
		}

		static void setHeapIndex(Node *n, unsigned int index) {
			n->fIndex = index;
		}

		static int sort(const Node *a, const Node *b) {
			if(a->f == b->f) {
				return a->g - b->g;
			}
			return a->f - b->f;
		}
	};

	struct SortOnSearchEffort {
		static unsigned int getHeapIndex(const Node *n) {
			return n->searchEffortIndex;
		}

		static void setHeapIndex(Node *n, unsigned int index) {
			n->searchEffortIndex = index;
		}

		static int sort(const Node *a, const Node *b) {
			if(a->searchEffort == b->searchEffort) {
				return a->fhat - b->fhat;
			}
			return a->searchEffort - b->searchEffort;
		}
	};

	struct SortOnFHat {
		static int compare(const Node *a, const Node *b) {
			if(a->fhat == b->fhat) {
				return a->f - b->f;
			}
			return a->fhat - b->fhat;
		}
	};

	AEES(const InstanceFileMap &args) : incumbent(NULL) {
		weight = args.doubleVal("AEES Weight");


	}

	void insert(unsigned int id, double g, double ghat, double h, double hhat, double f, double fhat, double d, double dhat, double searchEffort) {
		double oldBestFHatVal = open.isEmpty() ? std::numeric_limits<double>::infinity() : open.peekLeftmost()->fhat * weight;

		Node *n = pool.construct(Node(id, g, ghat, h, hhat, f, fhat, d, dhat, searchEffort));
		addToQueues(n);
		nodes[n->id] = n;

		double newBestFHatVal = open.isEmpty() ? std::numeric_limits<double>::infinity() : open.peekLeftmost()->fhat * weight;

		if(oldBestFHatVal != newBestFHatVal) {
			rebuildFocal();
		}
	}

	Node* getBest() {
		Node *best;
		for(best = selectNode(); best->f > incumbent->f; best = selectNode()) {}

		return best;
	}

	void foundGoal(unsigned int id) {
		if(incumbent == NULL || nodes[id]->f < incumbent->f) {
			incumbent = nodes[id];
			weight = nodes[id]->f / open.peekLeftmost()->f;
		}
	}

	Node* getBestFAndSyncQueues() {
		Node* best = cleanup.pop();
		open.remove(best);
		if(focal.inHeap(best))
			focal.remove(best);
		return best;
	}

	Node* getBestFHatAndSyncQueues() {
		Node* best = open.extractLeftmost();
		if(focal.inHeap(best))
			focal.remove(best);
		cleanup.remove(best);
		return best;
	}

	Node* getBestDHatAndSyncQueues() {
		Node* best = focal.pop();
		open.remove(best);
		cleanup.remove(best);
		return best;
	}

	Node* selectNode() {
		const Node* bestF = cleanup.isEmpty() ? NULL : cleanup.peek();
		const Node* bestFhat = open.isEmpty() ? NULL : open.peekLeftmost();
		const Node* bestD = focal.isEmpty() ? NULL : focal.peek();

		if(bestD->fhat <= weight * bestF->f) {
			return getBestDHatAndSyncQueues();
		} else if(bestFhat->fhat <= weight * bestF->f) {
			return getBestFHatAndSyncQueues();
		} else {
			return getBestFAndSyncQueues();
		}
	}

	void rebuildFocal() {
		double bound = open.isEmpty() ? std::numeric_limits<double>::infinity() : open.peekLeftmost()->fhat * weight;
		focal.clear();
		auto includeThis = [bound](Node* n) { return n != NULL && n->fhat <= bound; };
		std::vector<Node*> range = open.getContiguousRange(includeThis);
		focal.createFromVector(range);
	}

	void addToQueues(Node *n) {
		double bound = open.isEmpty() ? std::numeric_limits<double>::infinity() : open.peekLeftmost()->fhat * weight;
		open.insert(n);
		cleanup.push(n);
		if(n->fhat <= bound) {
			focal.push(n);
		}
	}

	// void search(const Domain &domain) {



	// 	while(!open.isEmpty()) {
	// 		double oldBestFHatVal = open.isEmpty() ? std::numeric_limits<double>::infinity() : open.peekLeftmost()->fhat * weight;

	// 		Node* n = selectNode(open, focal, cleanup);
	// 		//Node* n = selectNode_con(open, focal, cleanup);
	// 		//Node* n = selectNode_opt(open, focal, cleanup);

	// 		if(domain.isGoal(n->node->state)) {
	// 			incumbent = n;
	// 			this->goalCount++;
	// 			this->goal = n->node;
	// 			break;
	// 		}

	// 		vector<HeuristicNode *> children = domain.expand(n->node);
	// 		this->exp += 1;
	// 		this->gen += children.size();
	// 		for(auto child = children.begin(); child != children.end(); ++child) {
	// 			addToQueues(open, focal, cleanup, *child);
	// 		}

	// 		double newBestFHatVal = open.isEmpty() ? std::numeric_limits<double>::infinity() : open.peekLeftmost()->fhat * weight;

	// 		if(oldBestFHatVal != newBestFHatVal) {
	// 			rebuildFocal(open, focal);
	// 		}
	// 	}

	// }


	// virtual void output(FILE *f, const Domain &domain) const {
	// 	this->dfpair(f, "weight", weight);
	// 	Search<Domain>::output(f, domain);
	// }

	boost::object_pool<Node> pool;
	std::unordered_map<unsigned int, Node*> nodes;
	RBTree<Node, SortOnFHat> open;
	InPlaceBinaryHeap<Node, SortOnSearchEffort> focal;
	InPlaceBinaryHeap<Node, SortOnF> cleanup;

	double weight;
	Node* incumbent;
};