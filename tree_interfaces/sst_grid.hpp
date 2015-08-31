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
		double resizeThreshold, unsigned int persist = 100) :
		insertionInterface(insertionInterface), queryInterface(queryInterface), discretizationPercent(startingDiscretizationPercent),
		resizeThreshold(resizeThreshold), persist(persist) {
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

		if(didAddNewEdge) {
			added++;
			total++;
		} else {
			filtered++;
			total++;
			if(total >= persist && filtered / total >= resizeThreshold) {
				rebuildDiscretizationSizes(discretizationPercent * 0.5);
				rehashData();
			}
		}
	}

private:
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
		added = filtered = 0;
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
	unsigned int added, filtered, total, persist;
};