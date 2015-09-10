#pragma once

template<class Agent, class InsertionInteface, class QueryInterface>
class TreeInterface {
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;
public:
	TreeInterface(InsertionInteface &insertionInterface, QueryInterface &queryInterface) :
		insertionInterface(insertionInterface), queryInterface(queryInterface) {}

	std::pair<Edge*, State> getTreeSample() {
		return queryInterface.getTreeSample();
	}

	bool insertIntoTree(Edge *edge) {
		insertionInterface.insertPoint(edge);
		return true;
	}

	Edge *getTreeEdge(const State &s) const {
		return queryInterface.getTreeEdge(s);
	}

private:
	InsertionInteface &insertionInterface;
	QueryInterface &queryInterface;
};