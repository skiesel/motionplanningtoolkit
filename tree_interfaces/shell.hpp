#pragma once

template<class Agent, class InsertionInteface, class QueryInterface>
class Shell {
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;
public:
	Shell(InsertionInteface &insertionInterface, QueryInterface &queryInterface, double shellDepth) :
		insertionInterface(insertionInterface), queryInterface(queryInterface), shellDepth(shellDepth) {}

	std::pair<Edge*, State> getTreeSample() {
		return queryInterface.getTreeSample();
	}

	bool insertIntoTree(Edge *edge) {
		queryInterface.reached(edge, shellDepth);
		insertionInterface.insertPoint(edge);
		return true;
	}

	void removeFromTree(Edge *edge) {
		queryInterface.removed(edge);
		insertionInterface.removeFromTree(edge);
	}

	Edge *getTreeEdge(const State &s) const {
		return queryInterface.getTreeEdge(s);
	}

private:
	InsertionInteface &insertionInterface;
	QueryInterface &queryInterface;
	double shellDepth;
};