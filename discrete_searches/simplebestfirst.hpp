#pragma once

class SimpleBestFirst {
	struct Node {
		Node(unsigned int id, double val) : id(id), val(val) {}
		
		static bool HeapCompare(const Node *n1, const Node *n2) {
			return n1->val < n2->val;
		}

		unsigned int id;
		double val;
	};
public:
	SimpleBestFirst() {}

	void insert(unsigned int id, double val) {
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
};