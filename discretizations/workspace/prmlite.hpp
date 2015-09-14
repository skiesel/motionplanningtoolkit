#pragma once

#include <random>
#include <unordered_map>
#include <fcl/math/transform.h>

#ifdef VREPPLUGIN
#include "v_repLib.h"
#endif

#include "../../utilities/flannkdtreewrapper.hpp"
#include "../../utilities/datafile.hpp"
#include "../../utilities/math.hpp"

template <class Workspace, class Agent>
class PRMLite {
protected:
	typedef typename Agent::State State;
	typedef typename Agent::AbstractState AbstractState;
	typedef typename Agent::AbstractEdge AbstractEdge;

	struct Vertex {
		Vertex(const AbstractState &state, unsigned int id) : state(state), id(id) {}

		const std::vector<double> &getTreeStateVars() const {
			return state.getTreeStateVars();
		}

		void setPointIndex(unsigned int index) {
			// we aren't going to look things up in the traditional way so we don't need this stored
		}

		unsigned int id;
		AbstractState state;
	};

	struct Edge {
		enum CollisionCheckingStatus {
			UNKNOWN = 0,
			INVALID = 1,
			VALID = 2,
		};

		Edge() : status(UNKNOWN) {}

		Edge(unsigned int endpoint, double weight) : endpoint(endpoint), weight(weight), status(UNKNOWN) {

		}

		std::size_t operator()(const Edge &e) const {
			return e.endpoint;
		}

		bool operator==(const Edge &e) const {
			return e.endpoint == endpoint;
		}

		unsigned int endpoint;
		double weight;
		CollisionCheckingStatus status;
	};

	typedef flann::KDTreeIndexParams KDTreeType;
	typedef FLANN_KDTreeWrapper<KDTreeType, typename Agent::AbstractDistanceEvaluator, Vertex> KDTree;

public:
	PRMLite(const Workspace &workspace, const Agent &agent, unsigned int numVertices, unsigned int edgeSetSize, double collisionCheckDT, bool shouldGenerateEdges = true)
		: workspace(workspace),
		  agent(agent),
		  kdtree(KDTreeType(1), agent.getAbstractDistanceEvaluator(), agent.getTreeAbstractStateSize()),
		  collisionCheckDT(collisionCheckDT),
		  collisionChecks(0) {
		initialize(numVertices, edgeSetSize, shouldGenerateEdges);
	}

	void grow(unsigned int howManyToAdd) {
		fatal("PRM addMoreVertices not implemented");
	}

	unsigned int getCellCount() const {
		return vertices.size();
	}

	double getEdgeCostBetweenCells(unsigned int c1, unsigned int c2) const {
		auto vertexAndEdges = edges.find(c1);
		assert(vertexAndEdges != edges.end());

		const auto &edges = vertexAndEdges->second;

		auto edge = edges.find(c2);
		assert(edge != edges.end());

		return edge->second.weight;
	}

	unsigned int getCellId(const typename Agent::State &state) const {
		Vertex v(agent.toAbstractState(state), 0);
		auto res = kdtree.nearest(&v);

		assert(res.elements.size() > 0);

		return res.elements[0]->id;
	}

	std::vector<unsigned int> getNeighboringCells(unsigned int index) const {
		auto vertexAndEdges = edges.find(index);
		assert(vertexAndEdges != edges.end());

		const auto &edges = vertexAndEdges->second;

		std::vector<unsigned int> ids;
		ids.reserve(edges.size());
		for(const auto &edge : edges) {
			ids.push_back(edge.second.endpoint);
		}
		return ids;
	}

	typename Agent::State getRandomStateNearRegionCenter(unsigned int index, double radius) const {
		return agent.getRandomStateNearAbstractState(vertices[index]->state, radius);
	}

	void draw(bool drawPoints=true, bool drawLines=false, std::vector<std::vector<double>> colors = std::vector<std::vector<double>>()) const {
#ifdef WITHGRAPHICS
		drawOpenGL(drawPoints, drawLines, colors);
#endif
#ifdef VREPPLUGIN
		drawVREP(drawPoints, drawLines, colors);
#endif
	}

	bool edgeExists(unsigned int c1, unsigned int c2) const {
		auto vertexAndEdges = edges.find(c1);
		if(vertexAndEdges == edges.end()) {
			return false;
		}

		const auto &edges = vertexAndEdges->second;

		auto edge = edges.find(c2);
		return edge != edges.end();
	}

protected:

	void initialize(unsigned int numVertices, unsigned int edgeSetSize, bool shouldGenerateEdges) {
		dfpair(stdout, "prm vertex set size", "%lu", numVertices);
		dfpair(stdout, "prm edge set size", "%lu", edgeSetSize);
		dfpair(stdout, "prm edge collision check dt", "%g", collisionCheckDT);
		dfpair(stdout, "prm random quaternion z only", "%s", "true");

		startTime = clock();

		clock_t vertexStart = clock();

		generateVertices(numVertices);

		clock_t end = clock();
		double time = (double)(end-vertexStart) / CLOCKS_PER_SEC;
		dfpair(stdout, "prm vertex build time", "%g", time);

		if(shouldGenerateEdges) {
			clock_t edgeStart = clock();

			generateEdges(edgeSetSize);

			end = clock();

			time = (double)(end-edgeStart) / CLOCKS_PER_SEC;
			dfpair(stdout, "prm edge build time", "%g", time);
			dfpair(stdout, "prm collision checks", "%u", collisionChecks);


			time = (double)(end-startTime) / CLOCKS_PER_SEC;
			dfpair(stdout, "prm build time", "%g", time);
		}
	}

	void generateVertices(unsigned int numVertices) {
		vertices.reserve(numVertices);

		auto bounds = workspace.getBounds();

		AbstractState::getRandomAbstractState(bounds);

		while(vertices.size() < numVertices) {
			AbstractState state = AbstractState::getRandomAbstractState(bounds);

			if(workspace.safeAbstractState(agent, state)) {
				auto newVert = new Vertex(state, vertices.size());
				vertices.push_back(newVert);
				kdtree.insertPoint(newVert);
			}
		}
	}

	virtual void generateEdges(double edgeSetSize) {
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			auto res = kdtree.kNearest(vertices[i], edgeSetSize+1);

			for(const auto endVertex : res.elements) {

				if(edgeExists(i, endVertex->id)) {
					continue;
				}

				double cost = AbstractState::evaluateDistance(vertices[i]->state, endVertex->state);
				if(cost == 0) continue;

				AbstractEdge edgeCandidate = agent.generateAbstractEdge(vertices[i]->state, endVertex->state);

				if(edgeCandidate.size() != 0) {
					collisionChecks++;
				}

				if(edgeCandidate.size() == 0 || workspace.safeAbstractEdge(agent, edgeCandidate, collisionCheckDT)) {
					edges[i][endVertex->id] = Edge(endVertex->id, cost);
					edges[i][endVertex->id].status = Edge::CollisionCheckingStatus::VALID;

					if(agent.areAbstractEdgesSymmetric()) {
						edges[endVertex->id][i] = Edge(i, cost);
						edges[endVertex->id][i].status = Edge::CollisionCheckingStatus::VALID;
					}
				}
			}
		}
	}

#ifdef WITHGRAPHICS
	void drawOpenGL(bool drawPoints, bool drawLines, const std::vector<std::vector<double>> &colors) const {
		if(drawPoints) {
			glPointSize(1);
			unsigned int curIndex = 0;
			std::vector<double> white(3,1);
			for(const auto vert : vertices) {

				if(colors.size() == 0) {
					vert->state.draw();
					// agent.drawMesh(vert->transform);
				} else {
					OpenGLWrapper::Color color(colors[curIndex][0], colors[curIndex][1], colors[curIndex][2]);
					vert->state.draw(color);
					// drawOpenGLPoint(vert->transform.getTranslation(), colors[curIndex]);
					// OpenGLWrapper::Color color(colors[curIndex][0], colors[curIndex][1], colors[curIndex][2]);
					// agent.drawMesh(vert->transform, color);
					curIndex++;
				}
			}
			glPointSize(1);
		}

		// if(drawLines) {
		// 	OpenGLWrapper::Color color;

		// 	for(const auto& edgeSet : edges) {
		// 		// std::vector<double> edgeForVrep(6);
		// 		const auto &start = vertices[edgeSet.first]->state;

		// 		for(const auto& edge : edgeSet.second) {
		// 			const auto &end = vertices[edge.second.endpoint]->state;

		// 			AbstractEdge edgeCandidate = Agent::AbstractState::interpolate(start, end, collisionCheckDT);

		// 			for(const auto &s : edgeCandidate) {
		// 				s.draw();
		// 			}

		// 			// start.draw2DAbstractEdge(end, color);
		// 			// OpenGLWrapper::getOpenGLWrapper().drawLine(trans[0], trans[1], trans[2], trans2[0], trans2[1], trans2[2], color);
		// 		}
		// 	}
		// }
	}

	void drawOpenGLPoint(const fcl::Vec3f &point, const std::vector<double> &color) const {
		const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

		std::vector<double> pt;
		pt.push_back(point[0]);
		pt.push_back(point[1]);
		pt.push_back(point[2]);
		pt.push_back(1);
		pt.push_back(0);
		pt.push_back(0);
		pt.push_back(1);
		pt.push_back(1);
		pt.insert(pt.end(), color.begin(), color.end());
		pt.push_back(1); //the above color is a 3 vector, and we need alpha!
		pt.insert(pt.end(), identity.begin(), identity.end());

		OpenGLWrapper::getOpenGLWrapper().drawPoints(pt);
	}
#endif
#ifdef VREPPLUGIN
	void drawVREP(bool drawPoints, bool drawLines, const std::vector<std::vector<double>> &colors) const {
		// simFloat coords[12];
		// for(unsigned int i = 0; i < 6; ++i)
		// 	coords[i] = 0;

		// if(drawPoints) {
		// 	auto verticesHandle = simAddDrawingObject(sim_drawing_spherepoints | sim_drawing_itemcolors, 0.05, 0.0, -1, vertices.size(), NULL, NULL, NULL, NULL);
		// 	unsigned int curIndex = 0;
		// 	for(const auto vert : vertices) {
		// 		const auto& trans = vert->transform.getTranslation();


		// 		for(unsigned int i = 0; i < 3; ++i) {
		// 			coords[i] = trans[i];
		// 		}

		// 		if(colors.size() > 0) {
		// 			for(unsigned int i = 0; i < 3; ++i) {
		// 				coords[3+i] = colors[curIndex][i];
		// 			}
		// 		}
		// 		simAddDrawingObjectItem(verticesHandle, coords);
		// 		curIndex++;
		// 	}
		// }

		// if(drawLines) {
		// 	std::vector< std::vector<double> > edgesForVrep;
		// 	for(const auto& edgeSet : edges) {
		// 		std::vector<double> edgeForVrep(6);
		// 		const auto startVertex = vertices[edgeSet.first];

		// 		const auto& trans = startVertex->transform.getTranslation();
		// 		for(unsigned int i = 0; i < 3; ++i) {
		// 			edgeForVrep[i] = trans[i];
		// 		}

		// 		for(const auto& edge : edgeSet.second) {
		// 			const auto endVertex = vertices[edge.second.endpoint];

		// 			const auto& trans2 = endVertex->transform.getTranslation();
		// 			for(unsigned int i = 0; i < 3; ++i) {
		// 				edgeForVrep[3+i] = trans2[i];
		// 			}

		// 			edgesForVrep.push_back(edgeForVrep);
		// 		}
		// 	}

		// 	auto edgesHandle = simAddDrawingObject(sim_drawing_lines, 1, 0.0, -1, edgesForVrep.size(), NULL, NULL, NULL, NULL);

		// 	for(const auto &edge : edgesForVrep) {
		// 		for(unsigned int i = 0; i < 6; ++i)
		// 			coords[i] = edge[i];
		// 		simAddDrawingObjectItem(edgesHandle, coords);
		// 	}
		// }
	}

#endif
	const Workspace &workspace;
	const Agent &agent;
	std::vector<Vertex *> vertices;
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge>> edges;
	double collisionCheckDT;
	unsigned int collisionChecks;
	mutable KDTree kdtree;
	clock_t startTime;
};
