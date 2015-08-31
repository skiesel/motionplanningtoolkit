#pragma once

template<class Workspace, class Agent, class InsertionInteface, class QueryInterface>
class SST {
public:
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	struct Witness {
		Witness(Edge *edge, Edge *witness) : edge(edge), witness(witness) {}

		const typename Agent::StateVars &getTreeStateVars() const {
			return witness->getTreeStateVars();
		}
		int getPointIndex() const {
			return treeIndex;
		}
		void setPointIndex(int ptInd) {
			treeIndex = ptInd;
		}

		Edge *edge, *witness;
		int treeIndex;
	};

	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, Witness> KDTree;

	SST(const Workspace &workspace, const Agent &agent,
		InsertionInteface &insertionInterface, QueryInterface &queryInterface, double startingRadius, double resizeThreshold,
		unsigned int historySize = 100) : insertionInterface(insertionInterface), queryInterface(queryInterface),
		witnessInterface(KDTreeType(2), agent.getTreeStateSize()), radius(startingRadius), resizeThreshold(resizeThreshold),
		history(historySize), historyIndex(0), historyFails(0), historyFilled(false) {}

	Edge* getTreeSample() {
		return queryInterface.getTreeSample();
	}

	bool insertIntoTree(Edge *edge) {
		Witness *newWitness = new Witness(edge, edge);
		auto nearest = witnessInterface.nearestWithin(newWitness, radius);

		bool didAddNewEdge = false;

		if(nearest.elements.size() == 0 || nearest.distances[0] > radius) {
			witnessInterface.insertPoint(newWitness);
			insertionInterface.insertPoint(edge);
			didAddNewEdge = true;
		} else {
			Edge* prevBest = nearest.elements[0]->edge;
			if(edge->gCost() < prevBest->gCost()) {
				nearest.elements[0]->edge = edge;
				insertionInterface.removePoint(prevBest);
				insertionInterface.insertPoint(edge);
				didAddNewEdge = true;
			}
		}

		double failureRate = updateHistory(didAddNewEdge);
		if(historyFilled && failureRate >= resizeThreshold) {
			radius *= 0.5;
			resetHistory();
		}

		return didAddNewEdge;
	}

private:
	double updateHistory(bool success) {
		int nextIndex = historyIndex + 1;
		if(nextIndex >= history.size()) {
			nextIndex = 0;
			historyFilled = true;
		}

		if(historyFilled)
			historyFails -= history[historyIndex];

		history[historyIndex] = success ? 0 : 1;
		historyIndex += history[historyIndex];

		historyIndex = nextIndex;

		return (double)historyFails / (double)history.size();
	}

	void resetHistory() {
		historyFilled = historyIndex = historyFails = 0;
	}

	InsertionInteface &insertionInterface;
	QueryInterface &queryInterface;
	KDTree witnessInterface;
	double radius, resizeThreshold;
	std::vector<bool> history;
	int historyIndex, historyFails;
	bool historyFilled;
};