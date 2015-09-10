#pragma once

template<class Agent>
class FrequencyTreeInterface {
public:
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

private:
	struct Node {
<<<<<<< HEAD
<<<<<<< HEAD
		Node(Edge* e) : edge(e), frequency(0) {}

		Edge* edge;

		unsigned int heapIndex;
=======
		Node(Edge* e) : edge(e), frequency(0), heapIndex(std::numeric_limits<unsigned int>::max()) {}

>>>>>>> skiesel/master
=======
		Node(Edge* e) : edge(e), frequency(0), heapIndex(std::numeric_limits<unsigned int>::max()) {}

>>>>>>> skiesel/master
		int sort(const Node *n) const {
			return (frequency - n->frequency) > 0 ? -1 : 1;
		}
		unsigned int getHeapIndex() const {
			return heapIndex;
		}
		void setHeapIndex(unsigned int i) {
			heapIndex = i;
		}

<<<<<<< HEAD
<<<<<<< HEAD
		unsigned int frequency;
=======
		Edge* edge;
		unsigned int frequency;
		unsigned int heapIndex;
>>>>>>> skiesel/master
=======
		Edge* edge;
		unsigned int frequency;
		unsigned int heapIndex;
>>>>>>> skiesel/master
	};

public:

	FrequencyTreeInterface() {}

<<<<<<< HEAD
<<<<<<< HEAD
	Edge* getTreeSample() {
		Node *best = heap.peek();
		best->frequency++;
		heap.siftFromItem(best);
		return best->edge;
=======
=======
>>>>>>> skiesel/master
	std::pair<Edge*, State> getTreeSample() {
		Node *best = heap.peek();
		best->frequency++;
		heap.siftFromItem(best);
		fprintf(stderr, "FrequencyTreeInterface::getTreeSample returning edge->end as state to steer towards!");
		return std::make_pair(best->edge, best->edge->end);
<<<<<<< HEAD
>>>>>>> skiesel/master
=======
>>>>>>> skiesel/master
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