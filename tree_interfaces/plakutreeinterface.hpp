#pragma once

#include <limits>
#include <unordered_set>
#include "../utilities/flannkdtreewrapper.hpp"
#include "../utilities/heap.hpp"

template <class Workspace, class Agent, class Discretization>
class PlakuTreeInterface {

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, flann::L2<double>, VREPInterface::Edge> KDTree;
	typedef UniformSampler<Workspace, Agent, KDTree> UniformSampler;

	struct Region {
		Region(unsigned int id, const Agent& agent) : heapIndex(std::numeric_limits<unsigned int>::max()), id(id), numSelections(0), heuristic(std::numeric_limits<double>::infinity()), weight(0), onOpen(false) {
			KDTreeType kdtreeType;
			edgesInRegion = new KDTree(kdtreeType, agent.getTreeStateSize());
		}

		~Region() {
			delete edgesInRegion;
		}

		void setHeuristicAndPath(double heur, const std::vector<unsigned int> &path) {
			heuristic = heur;
			regionPath = path;
			regionPath.push_back(id);
		}

		void selected(double alpha) {
			numSelections++;
			weight = pow(alpha, numSelections) / (std::numeric_limits<double>::epsilon() + heuristic);
		}

		bool operator<(const Region &r) const {
			return weight < r.weight;
		}

		const unsigned int getRandomRegionAlongPathToGoal(std::uniform_real_distribution<double> &distribution,
			std::default_random_engine &generator) const {
			unsigned int randomIndex = (unsigned int)(distribution(generator) * regionPath.size());
			return regionPath[randomIndex];
		}

		/* used for initial heuristic computation */
		unsigned int heapIndex;
		int sort(const Region* r) const { return heuristic - r->heuristic; }
		unsigned int getHeapIndex() const { return heapIndex; }
		void setHeapIndex(unsigned int i) { heapIndex = i; }
		void updateRegionPath(const std::vector<unsigned int> &rp) { regionPath = rp; }


		unsigned int id, numSelections;
		double heuristic, weight;
		std::vector<unsigned int> regionPath;
		bool onOpen;
		KDTree *edgesInRegion;
	};

public:
	PlakuTreeInterface(const Workspace &workspace, const Agent &agent, Discretization& discretization,
		const State& start, const State& goal, double alpha, double b, double stateRadius) : agent(agent), workspace(workspace),
		discretization(discretization), startRegionId(discretization.getCellId(start)),
		goalRegionId(discretization.getCellId(goal)), alpha(alpha), b(b), stateRadius(stateRadius) {

		assert(alpha > 0 && alpha < 1);

		KDTreeType kdtreeType;
		uniformSamplerBackingKDTree = new KDTree(kdtreeType, agent.getTreeStateSize());

		uniformSampler = new UniformSampler(workspace, agent, *uniformSamplerBackingKDTree);

		unsigned int regionCount = discretization.getCellCount();
		regions.reserve(regionCount);
		for(unsigned int i = 0; i < regionCount; ++i) {
			regions.push_back(new Region(i, agent));
		}

		dijkstra(regions[startRegionId]);

		for(const auto region : regions) {
			fprintf(stderr, "%u\n", region->id);
			assert(region->regionPath.size() > 0);
		}

		regionHeap.push_back(regions[startRegionId]);
		regions[startRegionId]->onOpen = true;
	}

	~PlakuTreeInterface() {
		delete uniformSampler;
		delete uniformSamplerBackingKDTree;
	}

	State getTreeSample() {
		if(activeRegion != NULL) {
			activeRegion->selected(alpha);

			regionHeap.push_back(activeRegion);
			std::push_heap(regionHeap.begin(), regionHeap.end());
		}

		if(distribution(generator) < b) {
			activeRegion = regionHeap.front();
			std::pop_heap(regionHeap.begin(), regionHeap.end());
			regionHeap.pop_back();

			unsigned int regionAlongPath = activeRegion->getRandomRegionAlongPathToGoal(distribution, generator);
			return discretization.getRandomStateNearRegionCenter(regionAlongPath, stateRadius);
		} else {
			return uniformSampler->getTreeSample();
		}
	}

	void insertIntoTree(Edge* edge) {
		unsigned int newCellId = discretization.getCellId(edge->end);
		if(!regions[newCellId]->onOpen) {
			regionHeap.push_back(regions[newCellId]);
			std::push_heap(regionHeap.begin(), regionHeap.end());
			regions[newCellId]->onOpen = true;
		}

		regions[newCellId]->edgesInRegion->insertPoint(edge);
		uniformSamplerBackingKDTree->insertPoint(edge);
	}

private:

	struct DijkstraSearchNode {
		DijkstraSearchNode(unsigned int id, double cost) : id(id), cost(cost) {
			path.push_back(id);
		}

		DijkstraSearchNode(unsigned int id, double cost, const std::vector<unsigned int> &p) :
			id(id), cost(cost), path(p.begin(), p.end()) {
			path.push_back(id);
		}

		bool operator<(const DijkstraSearchNode &dsn) const {
			return cost > dsn.cost;
		}

		unsigned int id;
		double cost;
		std::vector<unsigned int> path;
	};

	void dijkstra(Region *region) {
		InPlaceBinaryHeap<Region> open;
		region->setHeuristicAndPath(0, std::vector<unsigned int>());
		open.push(region);

		while(!open.isEmpty()) {
			Region *current = open.pop();

			std::vector<unsigned int> kids = discretization.getNeighboringCells(current->id);
			for(unsigned int kid : kids) {
				double newHeuristic = current->heuristic + discretization.getEdgeCostBetweenCells(current->id, kid);
				Region *kidPtr = regions[kid];

				if(newHeuristic < kidPtr->heuristic) {
					kidPtr->heuristic = newHeuristic;
					kidPtr->setHeuristicAndPath(0, current->regionPath);

					if(open.inHeap(kidPtr)) {
						open.siftFromItem(kidPtr);
					} else {
						open.push(kidPtr);
					}
				}
			}
		}
	}

	const Workspace &workspace;
	const Agent &agent;
	const Discretization &discretization;
	
	UniformSampler *uniformSampler;
	KDTree *uniformSamplerBackingKDTree;

	unsigned int startRegionId, goalRegionId;
	std::vector<Region*> regions, regionHeap;
	Region *activeRegion;

	double alpha, b, stateRadius;
	std::uniform_real_distribution<double> distribution;
	mutable std::default_random_engine generator;
};