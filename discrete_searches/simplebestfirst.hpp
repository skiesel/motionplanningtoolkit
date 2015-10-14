#pragma once

class SimpleBestFirst {
	struct Node {
		Node(unsigned int id, double val) : id(id), val(val), heapIndex(std::numeric_limits<unsigned int>::max()) {}
		
		static bool pred(const Node *a, const Node *b) {
			return a->val < b->val;
		}
		static unsigned int getHeapIndex(const Node *n) {
			return n->heapIndex;
		}
		static void setHeapIndex(Node *n, unsigned int i) {
			n->heapIndex = i;
		}

		unsigned int id;
		double val;
		unsigned int heapIndex;
	};
public:
	SimpleBestFirst() {}

	void insert(unsigned int id, double val) {
		heap.push(new Node(id, val));
	}

	unsigned int peekBest() {
		return heap.peek()->id;
	}

	void updateBest(double val) {
		heap.peek()->val = val;
		heap.siftFromItem(heap.peek());
	}

	InPlaceBinaryHeap<Node, Node> heap;
};