#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>
#include "../utilities/datafile.hpp"
#include "../samplers/probabilitydensityfunction.hpp"

template<class Workspace, class Agent>
class AnytimeEST {
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

// 	struct Discretization {
// 		struct Node {
// 			Node(const State &state, unsigned int id) : id(id), state(state), inPDF(false) {}

// 			std::vector<double> getTreeStateVars() const {
// 				return state.getTreeStateVars();
// 			}

// 			void setPointIndex(unsigned int index) {
// 				pointIndex = index;
// 			}

// 			unsigned int getPointIndex() const {
// 				return pointIndex;
// 			}

// 			const State state;
// 			unsigned int id, pointIndex, inPDF;
// 		};

// 		typedef flann::KDTreeSingleIndexParams KDTreeType;
// 		typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, Node> KDTree;

// 		Discretization(const Workspace &workspace, const Agent &agent, unsigned int numRegions) :
// 		kdtree(KDTreeType(), agent.getDistanceEvaluator(), agent.getTreeStateSize()), nodes(numRegions), cellCount(numRegions) {
// 			unsigned int added = 0;
// 			while(added < cellCount) {
// 				State state = agent.getRandomState();
// 				// if(workspace.safeState(agent, state)) {
// 					nodes[added] = new Node(state, added);
// 					kdtree.insertPoint(nodes[added]);
// 					added++;
// 				// }
// 			}
// 		}

// 		unsigned int getCellCount() const {
// 			return cellCount;
// 		}

// 		unsigned int getCellId(const State &state) const {
// 			Node n(state, 0);
// 			auto res = kdtree.nearest(&n);
// 			return res.elements[0]->id;
// 		}

// 		void setInPDF(unsigned int index) { nodes[index]->inPDF = true; }

// #ifdef WITHGRAPHICS
// 		void draw() {
// 			auto blue = OpenGLWrapper::Color::Blue();
// 			auto green = OpenGLWrapper::Color::Green();
// 			for(auto n : nodes) {
// 				n->state.draw(n->inPDF ? blue : green);
// 			}
// 		}
// #endif
// 		KDTree kdtree;
// 		std::vector<Node*> nodes;
// 		unsigned int cellCount;
// 		bool inPDF;
// 	};

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


public:

	AnytimeEST(const Workspace &workspace, const Agent &agent, const InstanceFileMap &args, bool quiet = false,
		double gBound = std::numeric_limits<double>::infinity()) :
		workspace(workspace), agent(agent),
		quiet(quiet), gBound(gBound), incumbent(std::numeric_limits<double>::infinity()) {
		collisionCheckDT = args.doubleVal("Collision Check Delta t");
		goalBias = args.doubleVal("Goal Bias");
		howManySamplePoints = args.integerVal("Number of Sample Points");
		samplePointRadius = args.doubleVal("Sample Point Radius");

		if(!quiet) {
			dfpair(stdout, "collision check dt", "%g", collisionCheckDT);
			dfpair(stdout, "number of sample points", "%u", howManySamplePoints);
		}

		unsigned int regionCount = discretization.getCellCount();
		regions.reserve(regionCount);
		for(unsigned int i = 0; i < regionCount; ++i) {
			regions.push_back(new RegionNode(i));
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
		signal(SIGTERM, AnytimeEST::cleanup);
		boost::thread timer(AnytimeEST::timerFunction);


		unsigned int cellId = discretization.getCellId(start);
		typename ProbabilityDensityFunction<RegionNode>::Element *pdfElem = NULL;
		if(!regions[cellId]->inPDF) {
			auto root = pool.construct(start);
			regions[cellId]->addEdge(root);

			pdfElem = pdf.add(regions[cellId], regions[cellId]->getWeight()); discretization.setInPDF(cellId);
			regions[cellId]->setInPDF(true, pdfElem->getId());
		}

		dfrowhdr(stdout, "solution", 3, "cost", "length", "time");

		while(true) {

			RegionNode *randomNode = pdf.sample();
			Edge *randomEdge = randomNode->getRandomEdge(zeroToOne);

			for(unsigned int i = 0; i < howManySamplePoints; ++i) {
				State sample = (zeroToOne(GlobalRandomGenerator) < goalBias) ? goal : agent.getRandomStateNearState(randomEdge->end, samplePointRadius);

				samplesGenerated++;
				cellId = discretization.getCellId(sample);
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

							if(newEdgePointer->gCost() < incumbent && agent.isGoal(newEdgePointer->end, goal)) {
								std::vector<const Edge *> newSolution;
								newSolution.push_back(newEdgePointer);

								unsigned int edgeCount = 1;
								while(newSolution.back()->parent != NULL) {
									edgeCount++;
									newSolution.push_back(newSolution.back()->parent);
								}

								incumbent = newEdgePointer->gCost();
								dfrow(stdout, "solution", "gug", incumbent, edgeCount, (double)(clock()-startTime) / CLOCKS_PER_SEC, 0);
							}

							cellId = discretization.getCellId(newEdgePointer->end);
							regions[cellId]->addEdge(newEdgePointer);

							if(regions[cellId]->inPDF) {
								pdf.update(regions[cellId]->pdfId, regions[cellId]->getWeight());
							} else {
								pdfElem = pdf.add(regions[cellId], regions[cellId]->getWeight()); discretization.setInPDF(cellId);
								regions[cellId]->setInPDF(true, pdfElem->getId());
							}
						}
					}
				}
			}
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
	const Workspace &workspace;
	const Agent &agent;
	boost::object_pool<Edge> pool;
	std::vector<RegionNode*> regions;
	ProbabilityDensityFunction<RegionNode> pdf;
	Discretization discretization;

	double collisionCheckDT, goalBias, samplePointRadius;
	unsigned int howManySamplePoints;

	unsigned int samplesGenerated, samplesAdded, edgesGenerated, edgesAdded;

	bool quiet;
	double gBound, incumbent, timeout, startTime;
	std::uniform_real_distribution<double> zeroToOne;

	static AnytimeEST<Workspace, Agent> *selfPtr;
};

template<class Workspace, class Agent>
AnytimeEST<Workspace, Agent> *AnytimeEST<Workspace, Agent>::selfPtr = NULL;

