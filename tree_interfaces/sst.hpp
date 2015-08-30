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
		InsertionInteface &insertionInterface, QueryInterface &queryInterface, double startingRadius) :
		insertionInterface(insertionInterface), queryInterface(queryInterface),
		witnessInterface(KDTreeType(2), agent.getTreeStateSize()), radius(startingRadius) {}

	State getTreeSample() {
		return queryInterface.getTreeSample();
	}

	void insertIntoTree(Edge *edge) {
		Witness *newWitness = new Witness(edge, edge);
		auto nearest = witnessInterface.nearestWithin(newWitness, radius);

		if(nearest.elements.size() == 0 || nearest.distances[0] > radius) {
			witnessInterface.insertPoint(newWitness);
			insertionInterface.insertPoint(edge);
		} else {
			Edge* prevBest = nearest.elements[0]->edge;
			if(edge->gCost() < prevBest->gCost()) {
				nearest.elements[0]->edge = edge;
				insertionInterface.removePoint(prevBest);
				insertionInterface.insertPoint(edge);
			}
		}
	}

private:
	InsertionInteface &insertionInterface;
	QueryInterface &queryInterface;
	KDTree witnessInterface;
	double radius;
};