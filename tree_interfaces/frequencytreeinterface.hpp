#pragma once

template<class Agent>
class FrequencyTreeInterface {
public:
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

private:
	struct Node {
		Node(Edge* e) : edge(e), frequency(0), heapIndex(std::numeric_limits<unsigned int>::max()) {}

		int sort(const Node *n) const {
			return (frequency - n->frequency) > 0 ? -1 : 1;
		}
		unsigned int getHeapIndex() const {
			return heapIndex;
		}
		void setHeapIndex(unsigned int i) {
			heapIndex = i;
		}

		Edge* edge;
		unsigned int frequency;
		unsigned int heapIndex;
	};

public:

	FrequencyTreeInterface() {}

	Edge* getTreeSample() {
		Node *best = heap.peek();
		best->frequency++;
		heap.siftFromItem(best);
		return best->edge;
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
	InPlaceBinaryHeap<Node> heap;
};