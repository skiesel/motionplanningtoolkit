#pragma once

class SimpleBestFirst {
	struct Node {
<<<<<<< HEAD
<<<<<<< HEAD
		Node(unsigned int id, double val) : id(id), val(val) {}
		
		static bool HeapCompare(const Node *n1, const Node *n2) {
			return n1->val < n2->val;
=======
=======
>>>>>>> skiesel/master
		Node(unsigned int id, double val) : id(id), val(val), heapIndex(std::numeric_limits<unsigned int>::max()) {}
		
		int sort(const Node *n) const {
			return (val - n->val) > 0 ? -1 : 1;
		}
		unsigned int getHeapIndex() const {
			return heapIndex;
		}
		void setHeapIndex(unsigned int i) {
			heapIndex = i;
<<<<<<< HEAD
>>>>>>> skiesel/master
=======
>>>>>>> skiesel/master
		}

		unsigned int id;
		double val;
<<<<<<< HEAD
<<<<<<< HEAD
=======
		unsigned int heapIndex;
>>>>>>> skiesel/master
=======
		unsigned int heapIndex;
>>>>>>> skiesel/master
	};
public:
	SimpleBestFirst() {}

	void insert(unsigned int id, double val) {
<<<<<<< HEAD
<<<<<<< HEAD
		heap.push_back(new Node(id, val));
		std::push_heap(heap.begin(), heap.end(), Node::HeapCompare);
	}

	unsigned int getBest() {
		Node *front = heap.front();
		std::pop_heap(heap.begin(), heap.end(), Node::HeapCompare);
		heap.pop_back();

		unsigned int id = front->id;
		delete front;
		return id;
	}

	std::vector<Node*> heap;
=======
=======
>>>>>>> skiesel/master
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
<<<<<<< HEAD
>>>>>>> skiesel/master
=======
>>>>>>> skiesel/master
};