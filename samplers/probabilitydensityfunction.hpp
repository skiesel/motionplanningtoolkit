#pragma once

template<class Data>
class ProbabilityDensityFunction {
public:	
	struct Element {
		Element(Data *data, double weight, unsigned int index) : data(data),  weight(weight), index(index), leftSubtreeWeight(0), rightSubtreeWeight(0) {}

		Data* getData() const {
			return data;
		}

		unsigned int getId() const {
			return index;
		}

	protected:
		unsigned int leftChild() const {
			return index * 2;
		}
		
		unsigned int rightChild() const {
			return index * 2 + 1;
		}
		
		unsigned int parent() const {
			return index / 2;
		}
		
		bool isRoot() const {
			return index == 1;
		}

		bool isLeftChild() const {
			return index % 2 == 0;
		}

		void addToLeftSubtreeWeight(double w) {
			leftSubtreeWeight += w;
		}

		void addToRightSubtreeWeight(double w) {
			rightSubtreeWeight += w;
		}

		double getWeight() const {
			return weight;
		}

		double getLeftSubtreeWeight() const {
			return leftSubtreeWeight;
		}

		double getRightSubTreeWeight() const {
			return rightSubtreeWeight;
		}

		double getSubtreeWeight() const {
			return leftSubtreeWeight + weight + rightSubtreeWeight;
		}


	private:
		friend ProbabilityDensityFunction;
		Data *data;
		double weight, leftSubtreeWeight, rightSubtreeWeight;
		unsigned int index;
	};

	ProbabilityDensityFunction() : elements(1) {}

	Element* add(Data *data, double weight) {
		elements.push_back(new Element(data, weight, elements.size()));

		updateTree(elements.back(), weight);
		return elements.back();
	}

	Element* update(unsigned int index, double weight) {
		Element *element = elements[index];
		updateTree(element, weight - element->weight);
		element->weight = weight;
		return element;
	}

	Element* remove(unsigned int index) {
		Element *element = elements[index];

		if(index != elements.size()-1) {
			Element *replaceWith = elements.back();
			updateTree(replaceWith, -replaceWith->weight);

			updateTree(element, replaceWith->weight - element->weight);

			elements[index] = replaceWith;
			elements[index]->index = index;
			elements[index]->leftSubtreeWeight = element->leftSubtreeWeight;
			elements[index]->rightSubtreeWeight = element->rightSubtreeWeight;
		} else {
			updateTree(element, -element->weight);
		}

		elements.resize(elements.size() - 1);

		delete element;

		return index >= elements.size() ? NULL : elements[index];
	}

	Data* sample() const {
		double total = elements[1]->getSubtreeWeight();
		double value = zeroToOne(GlobalRandomGenerator) * total;

		Element *cur = elements[1];
		double left, weight, offset = 0;
		unsigned int kid;
		while(true) {

			weight = cur->getWeight();
			left = offset + cur->getLeftSubtreeWeight();

			if(value < left) {
				kid = cur->leftChild();
				if(kid >= elements.size()) return cur->data;
				cur = elements[kid];
			} else if(value < (left + weight)) {
				return cur->data;
			} else {
				kid = cur->rightChild();
				if(kid >= elements.size()) return cur->data;
				offset = left + weight;
				cur = elements[kid];
			}
		}
	}

	void checkInvariant() const {
		checkInvariant(elements[1]);
	}

private:

	void checkInvariant(Element *el) const {
		double leftSubtreeWeight = el->getLeftSubtreeWeight();
		double rightSubtreeWeight = el->getRightSubTreeWeight();

		double leftSubtreeWeight2 = el->leftChild() < elements.size() ? subtreeWeightSum(elements[el->leftChild()]) : 0;
		double rightSubtreeWeight2 = el->rightChild() < elements.size() ? subtreeWeightSum(elements[el->rightChild()]) : 0;

		assert(fabs(leftSubtreeWeight - leftSubtreeWeight2) < 0.000001);
		assert(fabs(rightSubtreeWeight - rightSubtreeWeight2) < 0.000001);

		if(el->leftChild() < elements.size()) checkInvariant(elements[el->leftChild()]);
		if(el->rightChild() < elements.size()) checkInvariant(elements[el->rightChild()]);
	}

	double subtreeWeightSum(Element *el) const {
		double left = el->leftChild() < elements.size() ? subtreeWeightSum(elements[el->leftChild()]) : 0;
		double right = el->rightChild() < elements.size() ? subtreeWeightSum(elements[el->rightChild()]) : 0;

		return left + el->weight + right;
	}

	void updateTree(Element* el, double weight) {
		Element *cur = el;
		Element *parent;

		while(!cur->isRoot()) {
			parent = elements[cur->parent()];
			if(cur->isLeftChild()) {
				parent->addToLeftSubtreeWeight(weight);
			} else {
				parent->addToRightSubtreeWeight(weight);
			}
			cur = parent;
		}
	}

	std::vector<Element*> elements;
	mutable std::uniform_real_distribution<double> zeroToOne;
};