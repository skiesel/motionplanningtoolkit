#pragma once

#include <limits>
#include <unordered_set>
#include "../utilities/flannkdtreewrapper.hpp"
#include "../utilities/heap.hpp"
#include "../discretizations/workspace/lazyprmlite.hpp"

template <class Workspace, class Agent, class Discretization> class DijkstraRunner;

template <class Workspace, class Agent, class Discretization>
class PlakuTreeInterface {

	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;
	class EdgeWrapper;

	typedef flann::KDTreeIndexParams KDTreeType;
	// typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, Edge> KDTree;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, EdgeWrapper> KDTree2;
	typedef UniformSampler<Workspace, Agent, KDTree> UniformSamplerT;

	struct EdgeWrapper {
		EdgeWrapper(Edge *edge) : edge(edge), treeIndex(0) {}

		/* needed for being inserted into NN datastructure */
		const typename Agent::StateVars &getTreeStateVars() const {
			return edge->getTreeStateVars();
		}

		int getPointIndex() const {
			return treeIndex;
		}

		void setPointIndex(int ptInd) {
			treeIndex = ptInd;
		}

		int treeIndex;
		Edge *edge;
	};

	struct Region {
		Region(unsigned int id, const Agent &agent) : heapIndex(std::numeric_limits<unsigned int>::max()), id(id), numSelections(0), heuristic(std::numeric_limits<double>::infinity()), weight(0), onOpen(false) {
			KDTreeType kdtreeType(1);
			edgesInRegion = new KDTree2(kdtreeType, agent.getDistanceEvaluator(), agent.getTreeStateSize());
		}

		~Region() {
			delete edgesInRegion;
		}

		void setHeuristicAndPath(double heur, const std::vector<unsigned int> &path) {
			heuristic = heur;
			regionPath = path;
			regionPath.push_back(id);
		}

		void addPathCandidate(double heur, const std::vector<unsigned int> &path) {
			if(heur < heuristic) {
				bestIndex = heuristicCandidates.size();
				heuristic = heur;
				regionPath = path;
				regionPath.push_back(id);
			}
			heuristicCandidates.push_back(heur);
			regionPathCandidates.emplace_back(path.begin(), path.end());
			regionPathCandidates.back().emplace_back(id);
		}

		void currentInvalidFindNextBestPath() {
			regionPathCandidates.erase(regionPathCandidates.begin() + bestIndex);
			heuristicCandidates.erase(heuristicCandidates.begin() + bestIndex);

			if(heuristicCandidates.size() == 0) {
				bestIndex = 0;
				heuristic = std::numeric_limits<double>::infinity();
				regionPath.resize(0);
				return;
			}

			heuristic = std::numeric_limits<double>::infinity();
			for(unsigned int i = 0; i < heuristicCandidates.size(); ++i) {
				if(heuristicCandidates[i] < heuristic) {
					heuristic = heuristicCandidates[i];
					bestIndex = i;
				}
			}
			regionPath = regionPathCandidates[bestIndex];
		}

		void selected(double alpha) {
			numSelections++;
			weight = pow(alpha, numSelections) / (std::numeric_limits<double>::epsilon() + heuristic);
		}

		unsigned int getRandomRegionAlongPathToGoal(std::uniform_real_distribution<double> &distribution) const {
			unsigned int randomIndex = (unsigned int)(distribution(GlobalRandomGenerator) * regionPath.size());
			return regionPath[randomIndex];
		}

		void insertEdge(Edge *edge) {
			EdgeWrapper *ew = new EdgeWrapper(edge);
			edgeWrapperLookup[edge] = ew;
			edgesInRegion->insertPoint(ew);
		}

		Edge* nearest(Edge *edge) {
			EdgeWrapper ew(edge);
			return edgesInRegion->nearest(&ew).elements[0]->edge;
		}

		Edge* getNearestEdgeInRegion(const State &s) const {
			Edge e(s);
			EdgeWrapper ew(&e);
			auto res = edgesInRegion->nearest(&ew, 0, 1);
			assert(res.elements.size() > 0);
			return res.elements[0]->edge;
		}

		void removeEdge(Edge *edge) {
			edgesInRegion->removePoint(edgeWrapperLookup[edge]);
			edgeWrapperLookup.erase(edge);
		}

		unsigned int getEdgeCount() const {
			return edgeWrapperLookup.size();
		}

		/* used for initial heuristic computation */
		unsigned int heapIndex;
		static bool pred(const Region *a, const Region *b) {
			return a->heuristic < b->heuristic;
		}
		static unsigned int getHeapIndex(const Region *r) {
			return r->heapIndex;
		}
		static void setHeapIndex(Region *r, unsigned int i) {
			r->heapIndex = i;
		}
		void updateRegionPath(const std::vector<unsigned int> &rp) {
			regionPath = rp;
		}

		static bool HeapCompare(const Region *r1, const Region *r2) {
			return r1->weight < r2->weight;
		}

		unsigned int id, numSelections;
		double heuristic, weight;
		std::vector<unsigned int> regionPath;

		unsigned int bestIndex;
		std::vector<double> heuristicCandidates;
		std::vector<std::vector<unsigned int>> regionPathCandidates;

		bool onOpen;
		KDTree2 *edgesInRegion;
		std::unordered_map<Edge*, EdgeWrapper*> edgeWrapperLookup;
	};

public:
	PlakuTreeInterface(const Workspace &workspace, const Agent &agent, Discretization &discretization,
	                   const State &start, const State &goal, double alpha, double b, double stateRadius, double goalBias) : agent(agent), workspace(workspace),
		discretization(discretization), startRegionId(discretization.getCellId(start)),
		goalRegionId(discretization.getCellId(goal)), activeRegion(NULL), alpha(alpha), b(b), stateRadius(stateRadius), goalBias(goalBias), goal(goal) {

		assert(alpha > 0 && alpha < 1);

		KDTreeType kdtreeType(1);
		uniformSamplerBackingKDTree = new KDTree(kdtreeType, agent.getDistanceEvaluator(), agent.getTreeStateSize());

		uniformSampler = new UniformSamplerT(workspace, agent, *uniformSamplerBackingKDTree);

		bool connected = false;
		while(!connected) {

			for(auto r : regions) {
				delete r;
			}
			regions.clear();

			unsigned int regionCount = discretization.getCellCount();
			regions.reserve(regionCount);
			for(unsigned int i = 0; i < regionCount; ++i) {
				regions.push_back(new Region(i, agent));
			}

			clock_t startT = clock();

			dijkstra(regions[goalRegionId]);

			double endT = clock();

			dfpair(stdout, "dijkstra search time", "%g", (endT-startT) / CLOCKS_PER_SEC);
	
			connected = regions[discretization.getCellId(start)]->regionPath.size() != 0;

			if(!connected) {
				fprintf(stderr, "... growing discretization ...\n");
				discretization.grow();
			}

			goalRegionId = discretization.getCellId(goal);
		}
	}

	~PlakuTreeInterface() {
		delete uniformSampler;
		delete uniformSamplerBackingKDTree;
	}

#ifdef WITHGRAPHICS
	void draw() {
		std::vector<std::vector<double>> colorLookup(regions.size());

		double min = 1000000000;
		double max = 0;
		for(unsigned int i = 0; i < regions.size(); ++i) {
			min = std::min(regions[i]->heuristic, min);
			if(!isinf(regions[i]->heuristic))
				max = std::max(regions[i]->heuristic, max);
		}

		for(unsigned int i = 0; i < regions.size(); ++i) {
			colorLookup[i] = OpenGLWrapper::getColor(min, max, regions[i]->heuristic);
		}

		discretization.draw(true, false, colorLookup);
	}

	void drawNextTreeSample() {
		static double maxHeuristicValue = 0;
		if(maxHeuristicValue == 0) {
			for(const auto r : regions) {
				if(r->heuristic > maxHeuristicValue) {
					maxHeuristicValue = r->heuristic;
				}
			}
		}

		if(distribution(GlobalRandomGenerator) < b) {
			assert(!regionHeap.empty());
			std::vector<Region*> saved;
			do {
				saved.push_back(regionHeap.front());
				std::pop_heap(regionHeap.begin(), regionHeap.end(), Region::HeapCompare);
				regionHeap.pop_back();
			} while(activeRegion->getEdgeCount() == 0);

			assert(saved.back()->getEdgeCount() != 0);

			unsigned int regionAlongPath = saved.back()->getRandomRegionAlongPathToGoal(distribution);

			if(regionAlongPath == goalRegionId && distribution(GlobalRandomGenerator) < goalBias) {
				goal.draw();
			} else {
				unsigned int cur = saved.back()->regionPath.front();
				for(auto r : saved.back()->regionPath) {
					discretization.drawEdge(cur, r, OpenGLWrapper::getColor(0, maxHeuristicValue, regions[r]->heuristic));
					cur = r;
					discretization.drawVertex(cur, OpenGLWrapper::getColor(0, maxHeuristicValue, regions[r]->heuristic));
				}

				double maxWeight = 0;
				for(const auto r : regionHeap) {
					if(r->weight > maxWeight) {
						maxWeight = r->weight;
					}
				}

				for(const auto r : regionHeap) {
					discretization.drawVertex(r->id, OpenGLWrapper::getColor(0, maxWeight, r->weight));
				}
			}

			for(auto r : saved) {
				regionHeap.push_back(r);
				std::push_heap(regionHeap.begin(), regionHeap.end(), Region::HeapCompare);
			}
		} else {
			uniformSampler->getTreeSample().second.draw();
		}
	}
#endif

	std::pair<Edge*, State> getTreeSample() {
		if(activeRegion != NULL) {
			if(!activeRegion->onOpen) {
				regionHeap.push_back(activeRegion);
				std::push_heap(regionHeap.begin(), regionHeap.end(), Region::HeapCompare);
				activeRegion->onOpen = true;
			}
		}

		if(distribution(GlobalRandomGenerator) < b) {
			assert(!regionHeap.empty());
			
			do {
				activeRegion = regionHeap.front();
				std::pop_heap(regionHeap.begin(), regionHeap.end(), Region::HeapCompare);
				regionHeap.pop_back();

				activeRegion->selected(alpha);
				activeRegion->onOpen = false;
			} while(activeRegion->getEdgeCount() == 0);

			assert(activeRegion->getEdgeCount() != 0);

			unsigned int regionAlongPath = activeRegion->getRandomRegionAlongPathToGoal(distribution);

			if(regionAlongPath == goalRegionId && distribution(GlobalRandomGenerator) < goalBias) {
				return std::make_pair(activeRegion->getNearestEdgeInRegion(goal), goal);
			}

			State p = discretization.getRandomStateNearRegionCenter(regionAlongPath, stateRadius);

			return std::make_pair(activeRegion->getNearestEdgeInRegion(p), p);
		} else {
			return uniformSampler->getTreeSample();
		}
	}

	bool insertIntoTree(Edge *edge) {
		unsigned int newCellId = discretization.getCellId(edge->end);

		if(!regions[newCellId]->onOpen) {
			regions[newCellId]->selected(alpha);
			regionHeap.push_back(regions[newCellId]);
			std::push_heap(regionHeap.begin(), regionHeap.end(), Region::HeapCompare);
			regions[newCellId]->onOpen = true;
		}

		regions[newCellId]->insertEdge(edge);
		uniformSamplerBackingKDTree->insertPoint(edge);

		return true;
	}

	Edge *getTreeEdge(const State &s) const {
		Edge edge(s);
		unsigned int cellId = discretization.getCellId(edge.end);
		return regions[cellId]->nearest(&edge);;
	}

	// typename KDTree::KNNResult nearest(const Edge *edge) const {
	// 	unsigned int cellId = discretization.getCellId(edge->end);
	// 	return regions[cellId]->nearest(edge);
	// }

	void removeFromTree(Edge *edge) {
		unsigned int cellId = discretization.getCellId(edge->end);
		regions[cellId]->removeEdge(edge);
		uniformSamplerBackingKDTree->removePoint(edge);
	}

private:
	void dijkstra(Region *region) {
		dijkstraRunner.dijkstra(*this, region);
	}

	const Workspace &workspace;
	const Agent &agent;
	Discretization &discretization;

	UniformSamplerT *uniformSampler;
	KDTree *uniformSamplerBackingKDTree;

	unsigned int startRegionId, goalRegionId;
	std::vector<Region *> regions, regionHeap;
	Region *activeRegion;

	double alpha, b, stateRadius;
	std::uniform_real_distribution<double> distribution;
	double goalBias;
	const State &goal;



	/* This is ugly, but I think slightly better than the alternatives */
	template<class W, class A, class D>
	class DijkstraRunner {
		typedef PlakuTreeInterface<W, A, D> TheBoss;
		typedef typename TheBoss::Region Region;
	public:
		void dijkstra(TheBoss &theBoss, Region *region) {
			InPlaceBinaryHeap<Region, Region> open;
			std::unordered_set<unsigned int> closed;
			region->setHeuristicAndPath(0, std::vector<unsigned int>());
			open.push(region);

			closed.insert(region->id);

			while(!open.isEmpty()) {
				Region *current = open.pop();

				closed.insert(current->id);

				std::vector<unsigned int> kids = theBoss.discretization.getNeighboringCells(current->id);
				for(unsigned int kid : kids) {
					if(closed.find(kid) != closed.end()) continue;

					double newHeuristic = current->heuristic + theBoss.discretization.getEdgeCostBetweenCells(current->id, kid);
					Region *kidPtr = theBoss.regions[kid];

					if(newHeuristic < kidPtr->heuristic) {
						kidPtr->setHeuristicAndPath(newHeuristic, current->regionPath);

						if(open.inHeap(kidPtr)) {
							open.siftFromItem(kidPtr);
						} else {
							open.push(kidPtr);
						}
					}
				}
			}
		}
	};


	//There is an issue in this code where some edges are being accepted as valid (because of the laziness)
	//without ever being collision checked 
	template<class W, class A>
	class DijkstraRunner<W, A, LazyPRMLite<Workspace, Agent> > {
		typedef PlakuTreeInterface<Workspace, Agent, LazyPRMLite<Workspace, Agent> > TheBoss;
		typedef typename TheBoss::Region Region;
	public:
		void dijkstra(TheBoss &theBoss, Region *region) {
			InPlaceBinaryHeap<Region, Region> open;
			std::unordered_set<unsigned int> closed;
			region->setHeuristicAndPath(0, std::vector<unsigned int>());
			open.push(region);
			closed.insert(region->id);

			while(!open.isEmpty()) {
				Region *current = open.peek();

				closed.insert(current->id);

				unsigned int pathLength = current->regionPath.size();

				if(pathLength > 2 && !theBoss.discretization.isValidEdge(current->regionPath[pathLength-1], current->regionPath[pathLength-2])) {
					current->currentInvalidFindNextBestPath();
					if(current->regionPath.size() > 0) {
						open.siftFromItem(current);
					} else {
						open.pop();
					}
					continue;
				}

				current = open.pop();

				std::vector<unsigned int> kids = theBoss.discretization.getNeighboringCells(current->id);
				for(unsigned int kid : kids) {
					if(closed.find(kid) != closed.end()) continue;

					double newHeuristic = current->heuristic + theBoss.discretization.getEdgeCostBetweenCells(current->id, kid);
					Region *kidPtr = theBoss.regions[kid];
					double oldHeuristic = kidPtr->heuristic;
					kidPtr->addPathCandidate(newHeuristic, current->regionPath);
					if(newHeuristic < oldHeuristic) {
						if(open.inHeap(kidPtr)) {
							open.siftFromItem(kidPtr);
						} else {
							open.push(kidPtr);
						}
					}
				}
			}

			// Get the edge checks to print
			theBoss.discretization.dfPairs();
		}
	};

	friend class DijkstraRunner<Workspace, Agent, Discretization>;
	DijkstraRunner<Workspace, Agent, Discretization> dijkstraRunner;
};