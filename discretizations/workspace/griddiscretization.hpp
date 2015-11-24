#pragma once

template <class Workspace, class Agent>
class GridDiscretization {

	typedef typename Agent::State State;
	typedef typename Agent::AbstractState AbstractState;
	typedef typename Agent::AbstractEdge AbstractEdge;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	struct Vertex {
		Vertex() {}
		Vertex(const AbstractState &state, const std::vector<unsigned int> &gridCoordinate, unsigned int id) :
			state(state), gridCoordinate(gridCoordinate), id(id) {}

		unsigned int id;
		std::vector<unsigned int> gridCoordinate;
		AbstractState state;
	};

	struct Edge {
		Edge() {}
		Edge(unsigned int endpoint, double weight) : endpoint(endpoint), weight(weight) {}

		unsigned int endpoint;
		double weight;
	};

public:
	GridDiscretization(const Workspace &workspace, const Agent &agent, double collisionCheckDT, double hintSize = 0.1) :
		workspace(workspace), agent(agent), collisionCheckDT(collisionCheckDT), cellCount(0) {

		stateVarRanges = agent.getAbstractStateVarRanges(workspace.getBounds());
		sizes = std::vector<double>(stateVarRanges.size(), hintSize);
		nextResize = 0;

		dimensionRanges.resize(stateVarRanges.size());
		for(unsigned int i = 0; i < dimensionRanges.size(); ++i) {
			dimensionRanges[i] = stateVarRanges[i].second - stateVarRanges[i].first;
		}

		discretizationSizes.resize(stateVarRanges.size());
		discreteDimensionSizes.resize(stateVarRanges.size());

		generateOffsets();

		reinitialize();
	}

	void grow() {
		sizes[nextResize] *= 0.5;
		if(++nextResize >= sizes.size()) nextResize = 0;
		reinitialize();
	}

	unsigned int getCellCount() const {
		return cellCount;
	}

	double getEdgeCostBetweenCells(unsigned int c1, unsigned int c2) const {
		auto vertexAndEdges = edges.find(c1);
		assert(vertexAndEdges != edges.end());

		const auto &edges = vertexAndEdges->second;

		auto edge = edges.find(c2);
		assert(edge != edges.end());

		return edge->second.weight;
	}

	unsigned int getCellId(const State &state) const {

		const auto abstractVars = agent.toAbstractState(state).getTreeStateVars();
		for(unsigned int i = 0; i < abstractVars.size(); ++i) {
			assert(abstractVars[i] >= stateVarRanges[i].first &&
				abstractVars[i] <= stateVarRanges[i].second);
		}
		return getIndex(abstractVars);
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

	State getRandomStateNearRegionCenter(unsigned int index, double radius) const {
		return agent.getRandomStateNearAbstractState(vertices[index].state, radius);
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

#ifdef WITHGRAPHICS
	void draw(bool drawPoints=true, bool drawLines=false, std::vector<std::vector<double>> colors = std::vector<std::vector<double>>(), const std::unordered_set<unsigned int> *includeThese = NULL) const {
		if(drawPoints) {
			glPointSize(5);
			unsigned int curIndex = 0;
			std::vector<double> white(3,1);
			for(const auto &vert : vertices) {
				if(includeThese != NULL) {
					if(includeThese->find(curIndex) == includeThese->end()) {
						curIndex++;
						continue;
					}
				}

				if(colors.size() == 0) {
					vert.state.draw();
					// agent.drawMesh(vert->transform);
				} else {
					OpenGLWrapper::Color color(colors[curIndex][0], colors[curIndex][1], colors[curIndex][2]);
					vert.state.draw(color);
					// drawOpenGLPoint(vert->transform.getTranslation(), colors[curIndex]);
					// OpenGLWrapper::Color color(colors[curIndex][0], colors[curIndex][1], colors[curIndex][2]);
					// agent.drawMesh(vert->transform, color);
					curIndex++;
				}
			}
			glPointSize(1);
		}

		if(drawLines) {
			OpenGLWrapper::Color color;

			for(const auto& edgeSet : edges) {
				// std::vector<double> edgeForVrep(6);
				const auto &start = vertices[edgeSet.first].state;

				const auto &startVars = start.getTreeStateVars();

				for(const auto& edge : edgeSet.second) {
					const auto &end = vertices[edge.second.endpoint].state;

					// AbstractEdge edgeCandidate = Agent::AbstractState::interpolate(start, end, collisionCheckDT);

					// for(const auto &s : edgeCandidate) {
					// 	s.draw();
					// }

					// start.draw2DAbstractEdge(end, color);
					// OpenGLWrapper::getOpenGLWrapper().drawLine(trans[0], trans[1], trans[2], trans2[0], trans2[1], trans2[2], color);

					const auto &endVars = end.getTreeStateVars();
					OpenGLWrapper::getOpenGLWrapper().drawLine(startVars[0], startVars[1], startVars[2], endVars[0], endVars[1], endVars[2], color);
				}
			}
		}
	}
#endif

private:
	void reinitialize() {

		for(unsigned int i = 0; i < discretizationSizes.size(); ++i) {
			discretizationSizes[i] = sizes[i] * dimensionRanges[i];
		}

		unsigned int newCellCount = 1;

		for(unsigned int i = 0; i < discreteDimensionSizes.size(); i++) {

			if(dimensionRanges[i] == 0) {
				discreteDimensionSizes[i] = 1;
			} else {
				discreteDimensionSizes[i] = ceil(dimensionRanges[i] / discretizationSizes[i]);
			}
			newCellCount *= discreteDimensionSizes[i];
		}

		generateVertices(cellCount, newCellCount);
		generateEdges();
	}

	void generateOffsets() {
		gridNeighbors.clear();
		std::vector<int> gridCoordinate(agent.getTreeAbstractStateSize(), 0);
		populateNeighborsWithDiagonals(gridCoordinate, 0);
		// populateNeighborsGrid();
	}

	void generateVertices(unsigned int oldCount, unsigned int newCount) {
		cellCount = newCount;

		vertices.reserve(cellCount);

		for(unsigned int i = 0; i < oldCount; ++i) {
			auto gridPoint = getGridCoordinates(i);
			auto point = getGridCenter(i);

			vertices[i].gridCoordinate = gridPoint;
			vertices[i].state = AbstractState::getAbstractState(point);
		}

		for(unsigned int i = oldCount; i < cellCount; ++i) {
			auto gridPoint = getGridCoordinates(i);
			auto point = getGridCenter(i);

			auto state = AbstractState::getAbstractState(point);
			vertices.emplace_back(state, gridPoint, i);
		}
	}

	void generateEdges() {
		edges.clear();

		for(unsigned int i = 0; i < vertices.size(); ++i) {
			edges[i] = std::unordered_map<unsigned int, Edge>();

			auto discreteCoordinate = getGridCoordinates(i);

			std::vector<unsigned int> neighbors = getNeighbors(discreteCoordinate);

			// fprintf(stderr, "%u /  %lu : %lu\n", i, vertices.size(), neighbors.size());

			for(auto neighbor : neighbors) {
				if(edgeExists(i, neighbor)) {
					continue;
				}

				AbstractEdge edgeCandidate = agent.generateAbstractEdge(vertices[i].state, vertices[neighbor].state);

				if(edgeCandidate.size() == 0 || workspace.safeAbstractEdge(agent, edgeCandidate, collisionCheckDT)) {

					double cost = AbstractState::evaluateDistance(vertices[i].state, vertices[neighbor].state);

					edges[i][neighbor] = Edge(neighbor, cost);

					if(agent.areAbstractEdgesSymmetric()) {
						edges[neighbor][i] = Edge(i, cost);
					}
				}
			}
		}
	}

	std::vector<unsigned int> getNeighbors(const std::vector<unsigned int> &discreteCoordinate) const {
		std::vector<unsigned int> neighbors;

		for(auto neighbor : gridNeighbors) {
			std::vector<unsigned int> coord(discreteCoordinate);
			bool valid = true;
			for(unsigned int i = 0; i < neighbor.size(); ++i) {
				if((coord[i] == 0 && neighbor[i] < 0) ||
					(coord[i] + neighbor[i] >= discreteDimensionSizes[i])) { valid = false; break; }
				coord[i] += neighbor[i];
			}
			if(valid) {
				neighbors.push_back(getIndex(coord));
			}
		}

		return neighbors;
	}

	void populateNeighborsGrid() {
		gridNeighbors.clear();

		std::vector<int> gridCoordinate(agent.getTreeAbstractStateSize(), 0);

		for(unsigned int i = 0; i < gridCoordinate.size(); ++i) {
			gridCoordinate[i] += 1;
			gridNeighbors.push_back(gridCoordinate);
			
			gridCoordinate[i] -= 2;

			gridNeighbors.push_back(gridCoordinate);
			gridCoordinate[i] += 1;
		}
	}

	void populateNeighborsWithDiagonals(std::vector<int> &gridCoordinate, unsigned int index) {
		if(index >= gridCoordinate.size()) return;

		populateNeighborsWithDiagonals(gridCoordinate, index+1);

		gridCoordinate[index] += 1;
		gridNeighbors.push_back(gridCoordinate);
		populateNeighborsWithDiagonals(gridCoordinate, index+1);
		
		gridCoordinate[index] -= 2;

		gridNeighbors.push_back(gridCoordinate);
		populateNeighborsWithDiagonals(gridCoordinate, index+1);

		gridCoordinate[index] += 1;
	}

	std::vector<double> getGridCenter(unsigned int n) const {
		std::vector<double> point;

		point.push_back(stateVarRanges[0].first + (double)(n % discreteDimensionSizes[0]) * discretizationSizes[0] + discretizationSizes[0]  * 0.5);
		if(discreteDimensionSizes.size() > 1) {
			unsigned int previousDimSizes = 1;
			for(unsigned int i = 1; i < discreteDimensionSizes.size(); i++) {
				previousDimSizes *= discreteDimensionSizes[i];
				point.push_back(stateVarRanges[i].first + (double)(n / previousDimSizes % discreteDimensionSizes[i]) * discretizationSizes[i] + discretizationSizes[1]  * 0.5);
			}
		}

		return point;
	}

	std::vector<unsigned int> getGridCoordinates(unsigned int n) const {
		std::vector<unsigned int> coordinate;

		coordinate.push_back(n % discreteDimensionSizes[0]);
		if(discreteDimensionSizes.size() > 1) {
			unsigned int previousDimSizes = 1;
			for(unsigned int i = 1; i < discreteDimensionSizes.size(); i++) {
				previousDimSizes *= discreteDimensionSizes[i];
				coordinate.push_back(n / previousDimSizes % discreteDimensionSizes[i]);
			}
		}

		return coordinate;
	}

	unsigned int getIndex(const std::vector<double> &point) const {
		unsigned int index = 0;

		for(unsigned int i = 0; i < discretizationSizes.size(); i++) {

			unsigned int which = floor((point[i] - stateVarRanges[i].first) / discretizationSizes[i]);

			double offset = 1;
			for(unsigned int j = 0; j < i; j++) {
				offset *= discreteDimensionSizes[j];
			}

			index += which * offset;
		}
		return index;
	}

	unsigned int getIndex(const std::vector<unsigned int> &gridCoordinate) const {
		unsigned int index = 0;

		for(unsigned int i = 0; i < discreteDimensionSizes.size(); i++) {

			double offset = 1;
			for(unsigned int j = 0; j < i; j++) {
				offset *= discreteDimensionSizes[j];
			}

			index += gridCoordinate[i] * offset;
		}
		return index;
	}

	const Workspace &workspace;
	const Agent &agent;
	double collisionCheckDT;
	StateVarRanges stateVarRanges;
	std::vector<double> sizes;
	unsigned int nextResize, cellCount;
	
	std::vector<unsigned int> discreteDimensionSizes;
	std::vector<double> discretizationSizes, dimensionRanges;

	std::vector< std::vector<int> > gridNeighbors;

	std::vector<Vertex> vertices;
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge>> edges;
};