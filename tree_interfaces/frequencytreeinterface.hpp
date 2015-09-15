#pragma once

template<class Agent>
class FrequencyTreeInterface {
public:
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

private:
	struct Node {
		Node(Edge* e) : edge(e), frequency(0), heapIndex(std::numeric_limits<unsigned int>::max()) {}

		static int sort(const Node *a, const Node *b) {
			return (a->frequency - b->frequency) > 0 ? -1 : 1;
		}
		static unsigned int getHeapIndex(const Node *n) {
			return n->heapIndex;
		}
		static void setHeapIndex(Node *n, unsigned int i) {
			n->heapIndex = i;
		}

		Edge* edge;
		unsigned int frequency;
		unsigned int heapIndex;
	};

public:

	FrequencyTreeInterface() {}

	std::pair<Edge*, State> getTreeSample() {
		Node *best = heap.peek();
		best->frequency++;
		heap.siftFromItem(best);
		fprintf(stderr, "FrequencyTreeInterface::getTreeSample returning edge->end as state to steer towards!");
		return std::make_pair(best->edge, best->edge->end);
	}

	Edge *getTreeEdge(const State &s) const {
		fatal("FrequencyTreeInterface::getTreeEdge not implemented");
		return NULL;
	}

	bool insertIntoTree(Edge *edge) {
		Node *node = new Node(edge);
		nodes.push_back(node);
		heap.push(node);
		return true;
	}

	void removeFromTree(Edge *edge) {
		fatal("FrequencyTreeInterface::removeFromTree not implemented");
	}

	std::vector<Node*> nodes;
	InPlaceBinaryHeap<Node, Node> heap;
};