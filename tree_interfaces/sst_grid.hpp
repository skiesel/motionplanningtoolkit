#pragma once

template<class Workspace, class Agent, class InsertionInteface, class QueryInterface>
class SST_Grid {
public:
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;
	typedef typename Agent::StateVars StateVars;
	typedef typename Agent::StateVarRanges StateVarRanges;

	SST_Grid(const Workspace &workspace, const Agent &agent,
		InsertionInteface &insertionInterface, QueryInterface &queryInterface, double startingDiscretizationPercent,
		double resizeThreshold, unsigned int historySize = 10) :
		insertionInterface(insertionInterface), queryInterface(queryInterface), discretizationPercent(startingDiscretizationPercent),
		resizeThreshold(resizeThreshold), history(historySize), historyIndex(0), historyFails(0), historyFilled(false) {
			stateVarRanges = agent.getStateVarRanges(workspace.getBounds());
			rebuildDiscretizationSizes(discretizationPercent);
		}

	std::pair<Edge*, State> getTreeSample() {
		return queryInterface.getTreeSample();
	}

	bool insertIntoTree(Edge *edge) {
		unsigned int key = computeLookup(edge);

		auto prevBestPair = witnessInterface.find(key);

		bool didAddNewEdge = false;

		if(prevBestPair == witnessInterface.end()) {
			witnessInterface[key] = edge;
			insertionInterface.insertIntoTree(edge);
			didAddNewEdge = true;
		} else {
			Edge *prevBest = prevBestPair->second;

			//make sure the previous best is still in the tree
			if(prevBest->getPointIndex() != 0 && edge->gCost() < prevBest->gCost()) {
				tree[prevBest->parent].erase(prevBest);
				removeSubtree(prevBest);

				witnessInterface[key] = edge;
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
			rebuildDiscretizationSizes(discretizationPercent * 0.5);
			rehashData();
		}

		return didAddNewEdge;
	}

private:
	double updateHistory(bool success) {
		int nextIndex = historyIndex + 1;

		if(historyFilled) {
			historyFails -= history[historyIndex];
		}

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

	void rehashData() {
		std::vector<Edge*> witnesses(witnessInterface.size());
		unsigned int index = 0;
		for(const auto &keyValPair : witnessInterface) {
			witnesses[index++] = keyValPair.second;
		}

		witnessInterface.clear();

		for(Edge* e : witnesses) {
			witnessInterface[computeLookup(e)] = e;
		}
	}

	unsigned int computeLookup(const Edge *edge) const {
		const StateVars &point = edge->getTreeStateVars();
		unsigned int index = 0;

		for(unsigned int i = 0; i < point.size(); i++) {

			unsigned int which = floor((point[i] - stateVarRanges[i].first) / discretizationSizes[i]);

			double offset = 1;
			for(unsigned int j = 0; j < i; j++) {
				offset *= gridDimensions[j];
			}

			index += which * offset;
		}
		return index;
	}

	void rebuildDiscretizationSizes(double newDiscretizationPercent) {
		resetHistory();

		discretizationPercent = newDiscretizationPercent;
		discretizationSizes.clear();
		gridDimensions.clear();

		for(auto varRange : stateVarRanges) {
			double range = varRange.second - varRange.first;
			discretizationSizes.emplace_back(range * discretizationPercent);
			if(range == 0) {
				gridDimensions.push_back(1);
			} else {
				gridDimensions.push_back(ceil(range / discretizationSizes.back()));
			}
		}
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
	std::unordered_map<unsigned int, Edge*> witnessInterface;
	StateVarRanges stateVarRanges;
	std::vector<double> discretizationSizes, gridDimensions;
	double discretizationPercent, resizeThreshold;
	std::vector<int> history;
	int historyIndex, historyFails;
	bool historyFilled;
	std::unordered_map<Edge*, std::unordered_set<Edge*>> tree;
};