#pragma once

#include "prmlite.hpp"

template <class Workspace, class Agent>
class LazyPRMLite : public PRMLite<Workspace, Agent> {
	typedef typename Agent::State State;
	typedef typename Agent::AbstractState AbstractState;
	typedef typename Agent::AbstractEdge AbstractEdge;
	typedef typename PRMLite<Workspace, Agent>::Edge Edge;

public:
	LazyPRMLite(const Workspace &workspace, const Agent &agent, unsigned int numVertices,
	            unsigned int edgeSetSize, double collisionCheckDT) :
		PRMLite<Workspace, Agent>(workspace, agent, numVertices, edgeSetSize, collisionCheckDT, false) {

		// startTime is set in the parent constructor call

		clock_t edgeStart = clock();

		generateEdges(edgeSetSize);

		clock_t end = clock();

		double time = (double)(end-edgeStart) / CLOCKS_PER_SEC;
		dfpair(stdout, "prm edge build time", "%g", time);

		time = (double)(end-this->startTime) / CLOCKS_PER_SEC;
		dfpair(stdout, "prm build time", "%g", time);
	}

	bool isValidEdge(unsigned int i, unsigned int j) {
		if(!this->edgeExists(i, j)) return false;

		auto &edge = this->edges[i][j];

		if(edge.status == Edge::UNKNOWN) {
			AbstractEdge edgeCandidate = this->agent.generateAbstractEdge(this->vertices[i]->state, this->vertices[j]->state);

			if(edgeCandidate.size() != 0) {
				this->collisionChecks++;
			}

			if(edgeCandidate.size() == 0 || this->workspace.safeAbstractEdge(this->agent, edgeCandidate, this->collisionCheckDT)) {
				this->edges[i][j].status = Edge::VALID;
				if(this->agent.areAbstractEdgesSymmetric()) {
					this->edges[j][i].status = Edge::VALID;
				}
			} else {
				this->edges[i][j].status = Edge::INVALID;
				if(this->agent.areAbstractEdgesSymmetric()) {
					this->edges[j][i].status = Edge::INVALID;
				}
			}
		}

		return edge.status == Edge::VALID;
	}

	void dfPairs() const {
		dfpair(stdout, "prm collision checks", "%u", this->collisionChecks);
	}

	virtual void generateEdges(double edgeSetSize) {
		for(unsigned int i = 0; i < this->vertices.size(); ++i) {
			auto res = this->kdtree.kNearest(this->vertices[i], edgeSetSize+1, 0, 1);

			for(const auto endVertex : res.elements) {

				if(this->edgeExists(i, endVertex->id)) {
					continue;
				}

				double cost = AbstractState::evaluateDistance(this->vertices[i]->state, endVertex->state);
				if(cost == 0) continue;

				this->edges[i][endVertex->id] = Edge(endVertex->id, cost);
				this->edges[endVertex->id][i] = Edge(i, cost); //the reverse interpolation would be symmetric
			}
		}
	}
};
