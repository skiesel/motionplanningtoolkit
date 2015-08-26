#pragma once

/* This shuffles around pointers to class T with the assumption that class T defines:

unsigned int getHeapIndex() const
void setHeapIndex(unsigned int i)
int sort(const T*) const [ -1 less, 0 equal, 1 greater ]
*/


#include <limits>
#include <cassert>
#include <vector>

template <class T> class InPlaceBinaryHeap {
public:
	InPlaceBinaryHeap(int size=100) : heap(size), fill(0) {

	}

	~InPlaceBinaryHeap() {

	}

	void push(T *data) {
		if(fill >= heap.size()) {
			heap.resize(heap.size() * 2);
		}
		heap[fill] = data;
		data->setHeapIndex(fill);
		fill++;
		siftUp(fill-1);
	}

	T *peek() {
		assert(fill > 0);
		return heap[0];
	}

	T *pop() {
		assert(fill > 0);
		T *ret_T = heap[0];
		swap(0,fill-1);
		fill--;
		siftDown(0);
		ret_T->setHeapIndex(std::numeric_limits<unsigned int>::max());
		return ret_T;
	}

	bool isEmpty() const {
		return fill <= 0;
	}

	int getFill() const {
		return fill;
	}

	bool inHeap(const T *data) const {
		return data->getHeapIndex() != std::numeric_limits<unsigned int>::max();
	}

	void siftFromItem(const T *data) {
		unsigned int index = data->getHeapIndex();
		if(index < fill) {
			int parent_index = parent(index);
			if(index > 0 && heap[parent_index]->sort(heap[index]) < 0)
				siftUp(index);
			else
				siftDown(index);
		}
	}


protected:
private:
	unsigned int left(unsigned int i) {
		return 2 * i;
	}

	unsigned int right(unsigned int i) {
		return 2 * i + 1;
	}

	unsigned int parent(unsigned int i) {
		return i / 2;
	}

	void swap(unsigned int i, unsigned int j) {
		T *temp = heap[i];
		heap[i] = heap[j];
		heap[j] = temp;

		heap[i]->setHeapIndex(i);
		heap[j]->setHeapIndex(j);
	}

	unsigned int childToSwap(unsigned int index) {
		unsigned int left_index = left(index);
		unsigned int right_index = right(index);
		if(left_index < fill && right_index < fill) {
			if(heap[left_index]->sort(heap[right_index]) > 0)
				return left_index;
			else
				return right_index;
		} else if(left_index < fill)
			return left_index;
		else if(right_index < fill)
			return right_index;
		else
			return index;
	}

	void siftDown(unsigned int index) {
		unsigned int child_index = childToSwap(index);
		if(heap[index]->sort(heap[child_index]) < 0) {
			swap(index, child_index);
			siftDown(child_index);
		}
	}

	void siftUp(unsigned int index) {
		if(index == 0) return;

		unsigned int parent_index = parent(index);
		if(heap[index]->sort(heap[parent_index]) > 0) {
			swap(index, parent_index);
			siftUp(parent_index);
		}
	}

	std::vector<T *> heap;
	unsigned int fill;
};
