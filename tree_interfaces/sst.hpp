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

	typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, Witness> KDTree;

	SST(const Workspace &workspace, const Agent &agent,
		InsertionInteface &insertionInterface, QueryInterface &queryInterface, double startingRadius, double resizeThreshold,
		unsigned int historySize = 100) : insertionInterface(insertionInterface), queryInterface(queryInterface),
		witnessInterface(KDTreeType(), agent.getDistanceEvaluator(), agent.getTreeStateSize()), radius(startingRadius), resizeThreshold(resizeThreshold),
		history(historySize), historyIndex(0), historyFails(0), historyFilled(false) {}

	std::pair<Edge*, State> getTreeSample() {
		return queryInterface.getTreeSample();
	}

	bool insertIntoTree(Edge *edge) {
		Witness *newWitness = new Witness(edge, edge);
		auto nearest = witnessInterface.nearestWithin(newWitness, radius);

		bool didAddNewEdge = false;

		if(nearest.elements.size() == 0 || nearest.distances[0] > radius) {
			witnessInterface.insertIntoTree(newWitness);
			insertionInterface.insertIntoTree(edge);
			didAddNewEdge = true;
		} else {
			Witness *witnessNode = nearest.elements[0];
			Edge *prevBest = witnessNode->edge;

			//make sure the previous best is still in the tree
			if(prevBest->getPointIndex() != 0 && edge->gCost() < prevBest->gCost()) {
				tree[prevBest->parent].erase(prevBest);
				removeSubtree(prevBest);

				witnessNode->edge = edge;
				insertionInterface.insertIntoTree(edge);
				didAddNewEdge = true;
			}
		}

		if(didAddNewEdge) {
			if(tree.find(edge->parent) == tree.end()) {
				tree[edge->parent] = std::unordered_set<Edge*>();
			}
			
			tree[edge->parent].insert(edge);
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

		if(historyFilled)
			historyFails -= history[historyIndex];

		if(nextIndex >= history.size()) {
			nextIndex = 0;
			historyFilled = true;
		}

		history[historyIndex] = success ? 0 : 1;
		historyFails += history[historyIndex];

		historyIndex = nextIndex;

		return (double)historyFails / (double)history.size();
	}

	void resetHistory() {
		historyFilled = historyIndex = historyFails = 0;
	}

	void removeSubtree(Edge *e) {
		for(Edge *kid : tree[e]) {
			removeSubtree(kid);
		}
		tree.erase(e);
		insertionInterface.removeFromTree(e);
	}

	InsertionInteface &insertionInterface;
	QueryInterface &queryInterface;
	KDTree witnessInterface;
	double radius, resizeThreshold;
	std::vector<bool> history;
	int historyIndex, historyFails;
	bool historyFilled;
	std::unordered_map<Edge*, std::unordered_set<Edge*>> tree;
};