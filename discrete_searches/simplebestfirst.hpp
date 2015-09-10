#pragma once

class SimpleBestFirst {
	struct Node {
		Node(unsigned int id, double val) : id(id), val(val), heapIndex(std::numeric_limits<unsigned int>::max()) {}
		
		int sort(const Node *n) const {
			return (val - n->val) > 0 ? -1 : 1;
		}
		unsigned int getHeapIndex() const {
			return heapIndex;
		}
		void setHeapIndex(unsigned int i) {
			heapIndex = i;
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

	InPlaceBinaryHeap<Node> heap;
};