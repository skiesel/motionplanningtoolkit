#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>
#include "../utilities/datafile.hpp"
#include "../samplers/probabilitydensityfunction.hpp"

template<class Workspace, class Agent>
class EST {
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
		Discretization(const Workspace &workspace, const Agent &agent, unsigned int numRegions) {
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

	EST(const Workspace &workspace, const Agent &agent, const InstanceFileMap &args, bool quiet = false,
		double gBound = std::numeric_limits<double>::infinity()) :
		workspace(workspace), agent(agent), discretization(workspace, agent, args.integerVal("Number of Regions")), quiet(quiet), gBound(gBound) {
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
	}

	std::vector<const Edge*> query(const State &start, const State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {
#ifdef WITHGRAPHICS
discretization.draw();
workspace.draw();
unsigned int iterations = 0;
#endif

		unsigned int cellId = discretization.getCellId(start);
		typename ProbabilityDensityFunction<RegionNode>::Element *pdfElem = NULL;
		if(!regions[cellId]->inPDF) {
			auto root = pool.construct(start);
			regions[cellId]->addEdge(root);

			pdfElem = pdf.add(regions[cellId], regions[cellId]->getWeight()); discretization.setInPDF(cellId);
			regions[cellId]->setInPDF(true, pdfElem->getId());
		}

		while(true) {
#ifdef WITHGRAPHICS
if(iterations++ > 100) { std::cin.ignore(); break; }
#endif
			RegionNode *randomNode = pdf.sample();
			Edge *randomEdge = randomNode->getRandomEdge(zeroToOne);

			for(unsigned int i = 0; i < howManySamplePoints; ++i) {
				State sample = (zeroToOne(GlobalRandomGenerator) < goalBias) ? goal : agent.getRandomStateNearState(randomEdge->end, samplePointRadius);

#ifdef WITHGRAPHICS
samples.push_back(sample);
#endif

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

#ifdef WITHGRAPHICS
treeEdges.push_back(newEdgePointer);
#endif

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
#ifdef WITHGRAPHICS
goto outtahere;
#endif
								return newSolution;
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

#ifdef WITHGRAPHICS
		outtahere:
		for(const Edge *edge : treeEdges) {
			edge->draw(OpenGLWrapper::Color::Red());
		}

		treeEdges.clear();

		for(const State &sample : samples) {
			sample.draw();
		}

		samples.clear();
#endif

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

	std::vector<const Edge *> treeEdges;
	std::vector<State> samples;

	double collisionCheckDT, goalBias, samplePointRadius;
	unsigned int howManySamplePoints;

	unsigned int samplesGenerated, samplesAdded, edgesGenerated, edgesAdded;

	bool quiet;
	double gBound;
	std::uniform_real_distribution<double> zeroToOne;
};