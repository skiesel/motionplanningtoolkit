#pragma once

#include <flann/flann.hpp>
#include <boost/pool/object_pool.hpp>
#include "../utilities/datafile.hpp"
#include "../samplers/probabilitydensityfunction.hpp"

template<class Workspace, class Agent>
class ESTBidirectional {
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
// 				if(workspace.safeState(agent, state)) {
// 					nodes[added] = new Node(state, added);
// 					kdtree.insertPoint(nodes[added]);
// 					added++;
// 				}
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


	typedef flann::KDTreeSingleIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::DistanceEvaluator, Edge> KDTree;

public:

	ESTBidirectional(const Workspace &workspace, const Agent &agent, const InstanceFileMap &args, bool quiet = false,
		double gBound = std::numeric_limits<double>::infinity()) :
		workspace(workspace), agent(agent), discretization(workspace, agent, args.integerVal("Number of Regions")),
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
	}

	std::vector<const Edge*> query(const State &start, const State &goal, int iterationsAtATime = -1, bool firstInvocation = true) {

#ifdef WITHGRAPHICS
discretization.draw();
workspace.draw();
unsigned int iterations = 0;
#endif

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

		while(true) {

#ifdef WITHGRAPHICS
if(iterations++ > 10) { std::cin.ignore(); break; }
#endif
			expand(forwardRegions, forwardPDF, forwardTree, forwardKDTree, goal);
			expand(backwardRegions, backwardPDF, backwardTree, backwardKDTree, start);
			
			auto solution = connection(forwardTree, backwardKDTree);
			if(solution.size() > 0) {
				if(!quiet) {
					dfpair(stdout, "solution cost", "%g", solution.back()->gCost());
					dfpair(stdout, "solution length", "%u", solution.size());
				}

				return solution;
			}
			
			// solution = connection(backwardTree, forwardKDTree);
			// if(solution.size() > 0) return solution;


		}

#ifdef WITHGRAPHICS
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

	std::vector<const Edge*> connection(const std::vector<Edge*> &edges, const KDTree &tree) {
		for(const auto edge : edges) {
			auto res = tree.kNearestWithin(edge, linkRadius);
			for(const auto e : res.elements) {
				Edge newEdge = agent.steer(edge->end, e->end, std::numeric_limits<double>::infinity());
				if(workspace.safeEdge(agent, newEdge, collisionCheckDT)) {
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
						newSolution.push_back(new Edge(*curEdge));
						newSolution.back()->updateParent(prevEdge);
					}

					std::vector<const Edge *> sol;
					sol.insert(sol.begin(), newSolution.begin(), newSolution.end());

					return sol;
				}
			}
		}

		return std::vector<const Edge*>();
	}

	void expand(std::vector<RegionNode*> &regions, ProbabilityDensityFunction<RegionNode> &pdf, std::vector<Edge*> &tree, KDTree &kdtree, const State &goal) {
		RegionNode *randomNode = pdf.sample();
		Edge *randomEdge = randomNode->getRandomEdge(zeroToOne);

		for(unsigned int i = 0; i < howManySamplePoints; ++i) {
			State sample = (zeroToOne(GlobalRandomGenerator) < goalBias) ? goal : agent.getRandomStateNearState(randomEdge->end, samplePointRadius);

#ifdef WITHGRAPHICS
samples.push_back(sample);
#endif

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

#ifdef WITHGRAPHICS
treeEdges.push_back(newEdgePointer);
#endif

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
	double gBound;
	std::uniform_real_distribution<double> zeroToOne;
};