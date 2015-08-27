#pragma once

template<class Agent, class InsertionInteface, class QueryInterface>
class SST {
public:
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

public:
	SST(InsertionInteface &insertionInterface, QueryInterface &queryInterface, double radius) :
		insertionInterface(insertionInterface), queryInterface(queryInterface), radius(radius) {}

	State getTreeSample() {
		return queryInterface.getTreeSample();
	}

	void insertIntoTree(Edge *edge) {
		auto nearest = insertionInterface.kNearestWithin(edge, radius, 1);

		bool insert = true;

		if(nearest.elements.size() > 0 || nearest.distances[0] <= radius) {
			
		}

		if(insert) {
			insertionInterface.insertPoint(edge);
		}
	}

private:
	InsertionInteface &insertionInterface;
	QueryInterface &queryInterface;
	double radius;
};