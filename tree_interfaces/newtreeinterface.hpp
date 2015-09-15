#pragma once

template<class Workspace, class Agent, class Discretization, class DiscreteSearch, class RegionManager>
class NewTreeInterface {
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;
public:
	struct Region {
		Region(unsigned int id) : id(id) {}

		void insertIntoRegion(Edge *edge) {
			manager.insertIntoTree(edge);
		}

		void removeFromRegion(Edge *edge) {
			//manager.removePoint(edge);
		}

		std::pair<Edge*, State> getSampleFromRegion() {
			return manager.getTreeSample();
		}

		/* used in dijkstra initialization */
		void setHeuristicAndPath(double heur, const std::vector<unsigned int> &path) {
			heuristic = heur;
			regionPath = path;
			regionPath.push_back(id);
		}

		/* used in dijkstra initialization */
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

		/* used in dijkstra initialization */
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

		/* used for initial heuristic computation */
		unsigned int heapIndex;
		static int sort(const Region *a, const Region *b) {
			return (a->heuristic - b->heuristic) > 0 ? -1 : 1;
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

		unsigned int id;
		RegionManager manager;
		double g, heuristic, weight;
		std::vector<unsigned int> regionPath;

		unsigned int bestIndex;
		std::vector<double> heuristicCandidates;
		std::vector<std::vector<unsigned int>> regionPathCandidates;
	};

	NewTreeInterface(const Workspace &workspace, const Agent &agent, Discretization &discretization,
	                   DiscreteSearch &discreteSearch, const State &start, const State &goal) : workspace(workspace),
		agent(agent), discretization(discretization), discreteSearch(discreteSearch) {

		unsigned int regionCount = discretization.getCellCount();
		regions.reserve(regionCount);
		for(unsigned int i = 0; i < regionCount; ++i) {
			regions.push_back(new Region(i));
		}

		unsigned int goalId = discretization.getCellId(goal);
		dijkstra(regions[goalId]);

		for(unsigned int i = 0; i < regionCount; ++i) {
			//greedy on h
			discreteSearch.insert(regions[i]->id, regions[i]->heuristic);
		}
	}

	std::pair<Edge*, State> getTreeSample() {
		return regions[discreteSearch.peekBest()]->getSampleFromRegion();
	}

	bool insertIntoTree(Edge *edge) {
		unsigned int cellId = discretization.getCellId(edge->end);
		regions[cellId]->insertIntoRegion(edge);
		return true;
	}

	Edge* getTreeEdge(const State &s) const {
		fatal("Not implemented: PlakuTreeInterface::getTreeEdge");
		return NULL;
	}

	void removeFromTree(Edge *edge) {
		unsigned int cellId = discretization.getCellId(edge->end);
		regions[cellId]->removeFromRegion(edge);
	}


private:
	void dijkstra(Region *region) {
		dijkstraRunner.dijkstra(*this, region);
	}

	const Workspace &workspace;
	const Agent &agent;
	Discretization &discretization;
	DiscreteSearch &discreteSearch;
	std::vector<Region*> regions;

	



	/* This is ugly, but I think slightly better than the alternatives */
	template<class W, class A, class D1, class D2, class R>
	class DijkstraRunner {
		typedef NewTreeInterface<W, A, D1, D2, R> TheBoss;
		typedef typename TheBoss::Region Region;
	public:
		void dijkstra(TheBoss &theBoss, Region *region) {
			InPlaceBinaryHeap<Region, Region> open;
			region->setHeuristicAndPath(0, std::vector<unsigned int>());
			open.push(region);

			while(!open.isEmpty()) {
				Region *current = open.pop();

				std::vector<unsigned int> kids = theBoss.discretization.getNeighboringCells(current->id);
				for(unsigned int kid : kids) {
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

	template<class W, class A, class D, class R>
	class DijkstraRunner<W, A, LazyPRMLite<Workspace, Agent>, D, R> {
		typedef NewTreeInterface<Workspace, Agent, LazyPRMLite<Workspace, Agent>, D, R > TheBoss;
		typedef typename TheBoss::Region Region;
	public:
		void dijkstra(TheBoss &theBoss, Region *region) {
			InPlaceBinaryHeap<Region, Region> open;
			region->setHeuristicAndPath(0, std::vector<unsigned int>());
			open.push(region);

			while(!open.isEmpty()) {
				Region *current = open.peek();

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

	friend class DijkstraRunner<Workspace, Agent, Discretization, DiscreteSearch, RegionManager>;
	DijkstraRunner<Workspace, Agent, Discretization, DiscreteSearch, RegionManager> dijkstraRunner;
};