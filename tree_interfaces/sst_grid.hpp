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
		double resizeThreshold, unsigned int historySize = 100) :
		insertionInterface(insertionInterface), queryInterface(queryInterface), discretizationPercent(startingDiscretizationPercent),
		resizeThreshold(resizeThreshold), history(historySize), historyIndex(0), historyFails(0), historyFilled(false) {
			stateVarRanges = agent.getStateVarRanges(workspace.getBounds());
			rebuildDiscretizationSizes(discretizationPercent);
		}

	State getTreeSample() {
		return queryInterface.getTreeSample();
	}

	void insertIntoTree(Edge *edge) {
		unsigned int key = computeLookup(edge);

		auto prevBestPair = witnessInterface.find(key);

		bool didAddNewEdge = false;

		if(prevBestPair == witnessInterface.end()) {
			witnessInterface[key] = edge;
			insertionInterface.insertPoint(edge);
			didAddNewEdge = true;
		} else {
			Edge *prevBest = prevBestPair->second;
			if(edge->gCost() < prevBest->gCost()) {
				witnessInterface[key] = edge;
				insertionInterface.removePoint(prevBest);
				insertionInterface.insertPoint(edge);
				didAddNewEdge = true;
			}
		}

		double failureRate = updateHistory(didAddNewEdge);
		if(historyFilled && failureRate >= resizeThreshold) {
			rebuildDiscretizationSizes(discretizationPercent * 0.5);
			rehashData();
		}
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

	InsertionInteface &insertionInterface;
	QueryInterface &queryInterface;
	std::unordered_map<unsigned int, Edge*> witnessInterface;
	StateVarRanges stateVarRanges;
	std::vector<double> discretizationSizes, gridDimensions;
	double discretizationPercent, resizeThreshold;
	std::vector<bool> history;
	int historyIndex, historyFails;
	bool historyFilled;
};