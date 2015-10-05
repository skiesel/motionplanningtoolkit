#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>
#include "../utilities/datafile.hpp"
#include "../samplers/probabilitydensityfunction.hpp"

template<class Workspace, class Agent>
class AnytimeBidirectionalEST {
	typedef typename Agent::State State;
	typedef typename Agent::Edge Edge;

	struct RegionNode {
		RegionNode(unsigned int id) : discretizationId(id), inPDF(false) {}

		Edge* getRandomEdge(std::uniform_real_distribution<double> &zeroToOne) const {
			unsigned int randomIndex = zeroToOne(GlobalRandomGenerator) * (double)edges.size();
			return edges[randomIndex];
		}

		double getWeight() const {
			return 1. / (double)edges.size();
		}

		void setInPDF(bool val, unsigned int id = 0) {
			inPDF = val;
			pdfId = id;
		}

		void addEdge(Edge *e) {
			edges.push_back(e);
		}

		unsigned int discretizationId, pdfId;
		bool inPDF;
		std::vector<Edge*> edges;
	};

	struct Discretization {
		Discretization() {
			discretization = 0.05;
			dimSize = 1 / discretization;
			cellCount = 1 / (discretization * discretization);
		}
		void setInPDF(unsigned int index) {}

		unsigned int getCellCount() const {
			return cellCount;
		}

		unsigned int getCellId(const State &s) const {
			const auto &vars = s.getStateVars();
			unsigned int x = vars[0] / discretization;
			unsigned int y = vars[1] / discretization;
			unsigned int index = x + y * dimSize;
			return index >= cellCount ? 0 : index;
		}
#ifdef WITHGRAPHICS
		void draw() {}
#endif

		double discretization, dimSize;
		unsigned int cellCount;
	};


	typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, Edge> KDTree;

public:

	AnytimeBidirectionalEST(const Workspace &workspace, const Agent &agent, const InstanceFileMap &args, bool quiet = false,
		double gBound = std::numeric_limits<double>::infinity()) :
		workspace(workspace), agent(agent),
		forwardKDTree(KDTreeType(), agent.getDistanceEvaluator(), agent.getTreeStateSize()), backwardKDTree(KDTreeType(), agent.getDistanceEvaluator(), agent.getTreeStateSize()),
		quiet(quiet), gBound(gBound) {
		
		collisionCheckDT = args.doubleVal("Collision Check Delta t");
		goalBias = args.doubleVal("Goal Bias");
		howManySamplePoints = args.integerVal("Number of Sample Points");
		samplePointRadius = args.doubleVal("Sample Point Radius");
		linkRadius = args.doubleVal("Link Radius");

		if(!quiet) {
			dfpair(stdout, "collision check dt", "%g", collisionCheckDT);
			dfpair(stdout, "number of sample points", "%u", howManySamplePoints);
		}

		unsigned int regionCount = discretization.getCellCount();
		forwardRegions.reserve(regionCount);
		backwardRegions.reserve(regionCount);
		for(unsigned int i = 0; i < regionCount; ++i) {
			forwardRegions.push_back(new RegionNode(i));
			backwardRegions.push_back(new RegionNode(i));
		}

		samplesGenerated = edgesGenerated = samplesAdded = edgesAdded = 0;

		timeout = args.doubleVal("Timeout");

		selfPtr = this;
	}

	static void cleanup(int param) {
		selfPtr->dfpairs();
		dffooter(stdout);
		exit(0);
	}

	static void timerFunction() {
		boost::this_thread::sleep(boost::posix_time::milliseconds(selfPtr->timeout * 1000));
		raise(SIGTERM);
	}

	std::vector<const Edge*> query(const State &start, const State &goal, double startT) {
		startTime = startT;
		signal(SIGTERM, AnytimeBidirectionalEST::cleanup);
		boost::thread timer(AnytimeBidirectionalEST::timerFunction);

		unsigned int cellId = discretization.getCellId(start);
		if(!forwardRegions[cellId]->inPDF) {
			typename ProbabilityDensityFunction<RegionNode>::Element *pdfElem = NULL;
			auto root = pool.construct(start);
			forwardRegions[cellId]->addEdge(root);

			pdfElem = forwardPDF.add(forwardRegions[cellId], forwardRegions[cellId]->getWeight()); discretization.setInPDF(cellId);
			forwardRegions[cellId]->setInPDF(true, pdfElem->getId());
			forwardTree.push_back(root);
			forwardKDTree.insertPoint(root);

			cellId = discretization.getCellId(goal);
			root = pool.construct(goal);
			backwardRegions[cellId]->addEdge(root);

			pdfElem = backwardPDF.add(backwardRegions[cellId], backwardRegions[cellId]->getWeight()); discretization.setInPDF(cellId);
			backwardRegions[cellId]->setInPDF(true, pdfElem->getId());
			backwardTree.push_back(root);
			backwardKDTree.insertPoint(root);
		}


		dfrowhdr(stdout, "solution", 3, "cost", "length", "time");
		double best = std::numeric_limits<double>::infinity();
		while(true) {

			expand(forwardRegions, forwardPDF, forwardTree, forwardKDTree, goal);
			expand(backwardRegions, backwardPDF, backwardTree, backwardKDTree, start);
			
			auto solution = connection(forwardTree, backwardKDTree);
			if(solution.size() > 0 && solution.back()->gCost() < best) {
				best = solution.back()->gCost();
				dfrow(stdout, "solution", "gug", solution.back()->gCost(), solution.size(), (double)(clock()-startTime) / CLOCKS_PER_SEC, 0);
			}
			
			// solution = connection(backwardTree, forwardKDTree);
			// if(solution.size() > 0) return solution;


		}

		return std::vector<const Edge*>();
	}

	void dfpairs() const {
		dfpair(stdout, "samples generated", "%u", samplesGenerated);
		dfpair(stdout, "samples rejected", "%u", samplesGenerated - samplesAdded);
		dfpair(stdout, "edges generated", "%u", edgesGenerated);
		dfpair(stdout, "edges rejected", "%u", edgesGenerated - edgesAdded);
	}

private:

	std::vector<const Edge*> connection(const std::vector<Edge*> &edges, const KDTree &tree) {
		std::vector<const Edge*> solution;
		double best = std::numeric_limits<double>::infinity();

		for(const auto edge : edges) {
			auto res = tree.kNearestWithin(edge, linkRadius);
			for(const auto e : res.elements) {
				Edge newEdge = agent.steer(edge->end, e->end, std::numeric_limits<double>::infinity());
				if(workspace.safeEdge(agent, newEdge, collisionCheckDT)) {
					double newSolutionCost = edge->gCost() + e->gCost();
					if(newSolutionCost >= best) {
						continue;
					}

					best = newSolutionCost;

					std::vector<Edge *> newSolution;
					newSolution.push_back(edge);

					unsigned int edgeCount = 1;
					while(newSolution.back()->parent != NULL) {
						edgeCount++;
						newSolution.push_back(newSolution.back()->parent);
					}

					std::reverse(newSolution.begin(), newSolution.end());

					newEdge.updateParent(edge);

					auto curEdge = e;
					newSolution.push_back(curEdge);

					while(curEdge->parent != NULL) {
						auto prevEdge = newSolution.back();

						curEdge = curEdge->parent;
						auto saved = new Edge(*curEdge);
						saved->updateParent(prevEdge);
						newSolution.push_back(saved);
					}

					solution.insert(solution.begin(), newSolution.begin(), newSolution.end());
				}
			}
		}

		return solution;
	}

	void expand(std::vector<RegionNode*> &regions, ProbabilityDensityFunction<RegionNode> &pdf, std::vector<Edge*> &tree, KDTree &kdtree, const State &goal) {
		RegionNode *randomNode = pdf.sample();
		Edge *randomEdge = randomNode->getRandomEdge(zeroToOne);

		for(unsigned int i = 0; i < howManySamplePoints; ++i) {
			State sample = (zeroToOne(GlobalRandomGenerator) < goalBias) ? goal : agent.getRandomStateNearState(randomEdge->end, samplePointRadius);

			samplesGenerated++;
			unsigned int cellId = discretization.getCellId(sample);
			if(zeroToOne(GlobalRandomGenerator) < regions[cellId]->getWeight()) {
				if(workspace.safeState(agent, sample)) {
					samplesAdded++;
					Edge newEdge = agent.steer(randomEdge->end, sample, std::numeric_limits<double>::infinity());
					edgesGenerated++;
					newEdge.updateParent(randomEdge);
					if(newEdge.gCost() >= gBound) {
						continue;
					}

					if(workspace.safeEdge(agent, newEdge, collisionCheckDT)) {
						edgesAdded++;
						Edge *newEdgePointer = pool.construct(newEdge);

						cellId = discretization.getCellId(newEdgePointer->end);
						regions[cellId]->addEdge(newEdgePointer);

						tree.push_back(newEdgePointer);
						kdtree.insertPoint(newEdgePointer);

						if(regions[cellId]->inPDF) {
							pdf.update(regions[cellId]->pdfId, regions[cellId]->getWeight());
						} else {
							typename ProbabilityDensityFunction<RegionNode>::Element *pdfElem = pdf.add(regions[cellId], regions[cellId]->getWeight()); discretization.setInPDF(cellId);
							regions[cellId]->setInPDF(true, pdfElem->getId());
						}
					}
				}
			}
		}
	}


	/*


if(agent.isGoal(newEdgePointer->end, goal)) {
								std::vector<const Edge *> newSolution;
								newSolution.push_back(newEdgePointer);

								unsigned int edgeCount = 1;
								while(newSolution.back()->parent != NULL) {
									edgeCount++;
									newSolution.push_back(newSolution.back()->parent);
								}

								if(!quiet) {
									dfpair(stdout, "solution cost", "%g", newEdgePointer->gCost());
									dfpair(stdout, "solution length", "%u", edgeCount);
								}
						
								std::reverse(newSolution.begin(), newSolution.end());
								
								return newSolution;
							}

	*/

	const Workspace &workspace;
	const Agent &agent;
	boost::object_pool<Edge> pool;
	std::vector<RegionNode*> forwardRegions, backwardRegions;
	ProbabilityDensityFunction<RegionNode> forwardPDF, backwardPDF;
	Discretization discretization;

	std::vector<Edge*> forwardTree, backwardTree;
	KDTree forwardKDTree, backwardKDTree;

	std::vector<const Edge *> treeEdges;
	std::vector<State> samples;

	double collisionCheckDT, goalBias, samplePointRadius, linkRadius;
	unsigned int howManySamplePoints;

	unsigned int samplesGenerated, samplesAdded, edgesGenerated, edgesAdded;

	bool quiet;
	double gBound, startTime, timeout;
	std::uniform_real_distribution<double> zeroToOne;

	static AnytimeBidirectionalEST<Workspace, Agent> *selfPtr;
};

template<class Workspace, class Agent>
AnytimeBidirectionalEST<Workspace, Agent> *AnytimeBidirectionalEST<Workspace, Agent>::selfPtr = NULL;

