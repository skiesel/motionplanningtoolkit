#pragma once

/* This shuffles around pointers to class T with the assumption that class Ops defines:

unsigned int getHeapIndex() const
void setHeapIndex(unsigned int i)
int sort(const T*) const [ -1 less, 0 equal, 1 greater ]
*/


#include <limits>
#include <cassert>
#include <vector>

template <class T, class Ops>
class InPlaceBinaryHeap {
public:
	InPlaceBinaryHeap(int size=100) : heap(size), fill(0) {}

	~InPlaceBinaryHeap() {}

	void push(T *data) {
		if(fill >= heap.size()) {
			heap.resize(heap.size() * 2);
		}
		heap[fill] = data;
		Ops::setHeapIndex(data, fill+1);
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
		Ops::setHeapIndex(ret_T, std::numeric_limits<unsigned int>::max());
		return ret_T;
	}

	bool isEmpty() const {
		return fill <= 0;
	}

	int getFill() const {
		return fill;
	}

	bool inHeap(const T *data) const {
		unsigned int index = Ops::getHeapIndex(data);
		return index > 0 && index <= fill;
	}

	void siftFromItem(const T *data) {
		unsigned int index = Ops::getHeapIndex(data);
		if(index < fill) {
			int parent_index = parent(index);
			if(index > 0 && Ops::sort(heap[index], heap[parent_index]) < 0)
				siftUp(index);
			else
				siftDown(index);
		}
	}

	void remove(const T* data) {
		unsigned int index = Ops::getHeapIndex(data);
		swap(index,fill);
		fill--;
		if(index <= fill) {
			int parent_index = parent(index);
			if(index > 1 && Ops::sort(heap[index], heap[parent_index]) < 0) {
				siftUp(index);
			}
			else {
				siftDown(index);
			}
		}
	}


protected:
private:
	inline unsigned int left(unsigned int i) {
		return (2 * i);
	}

	inline unsigned int right(unsigned int i) {
		return (2 * i + 1);
	}

	inline unsigned int parent(unsigned int i) {
		return (i / 2);
	}

	void swap(unsigned int i, unsigned int j) {
		T *temp = heap[i];
		heap[i] = heap[j];
		heap[j] = temp;

		Ops::setHeapIndex(heap[i], i);
		Ops::setHeapIndex(heap[j], j);
	}

	unsigned int childToSwap(unsigned int index) {
		unsigned int left_index = left(index);
		unsigned int right_index = right(index);
		if(left_index < fill && right_index < fill) {
			if(Ops::sort(heap[left_index], heap[right_index]) < 0)
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
		if(Ops::sort(heap[child_index], heap[index]) < 0) {
			swap(index, child_index);
			siftDown(child_index);
		}
	}

	void siftUp(unsigned int index) {
		if(index == 0) return;

		unsigned int parent_index = parent(index);
		if(Ops::sort(heap[index], heap[parent_index]) > 0) {
			swap(index, parent_index);
			siftUp(parent_index);
		}
	}

	std::vector<T *> heap;
	unsigned int fill;
};
