#pragma once

#include <vector>

#include "../utilities/instancefilemap.hpp"

template<typename Agent>
class RectangleMap2D {
	typedef typename Agent::Edge Edge;
	typedef typename Agent::State State;
	typedef typename Agent::StateVars StateVars;
	typedef typename Agent::AbstractState AbstractState;
	typedef typename Agent::AbstractEdge AbstractEdge;

	struct Rectangle {
		Rectangle(const std::vector<double> &vertList, unsigned int begin, unsigned int end) :
		vertices(vertList.begin() + begin, vertList.begin() + end), minx(std::numeric_limits<double>::infinity()),
		miny(std::numeric_limits<double>::infinity()) {


			for(unsigned int i = 0; i < vertices.size(); i+=2) {
				if(vertices[i] < minx) {
					minx = vertices[i];
				}
				if(vertices[i+1] < miny) {
					miny = vertices[i+1];
				}
			}

			double width = 0, height = 0;
			for(unsigned int i = 0; i < vertices.size(); i+=2) {
				double w = vertices[i] - minx;
				double h = vertices[i+1] - miny;
				if(w > width) {
					width = w;
				}
				if(h > height) {
					height = h;
				}
			}

			maxx = minx + width;
			maxy = miny + height;
		}

		bool contains(const StateVars &vars) const {
			assert(vars.size() == 2);

			return vars[0] >= minx && vars[0] <= maxx &&
					vars[1] >= miny && vars[1] <= maxy;
		}

#ifdef WITHGRAPHICS
		void draw() const {
			for(unsigned int i = 0; i < 4; ++i) {
				unsigned int index1 = i * 2;
				unsigned int index2 = ((i+1) % 4) * 2;

				OpenGLWrapper::getOpenGLWrapper().drawLine(
					vertices[index1], vertices[index1+1], 0,
					vertices[index2], vertices[index2+1], 0,
					OpenGLWrapper::Color::Red());
			}
		}
#endif

		std::vector<double> vertices;
		double minx, miny, maxx, maxy;
	};

public:
	typedef std::vector<std::pair<double, double> > WorkspaceBounds;

	RectangleMap2D(const InstanceFileMap &args) : workspaceBounds(2, std::make_pair(0,1)) {
		auto obsList = args.doubleList("Obstacles");
		unsigned int rectangleCount = obsList.size() / 8;
		for(unsigned int i = 0; i < rectangleCount; ++i) {
			obstacles.emplace_back(obsList, i * 8, (i+1)*8);
		}
	}

	const WorkspaceBounds& getBounds() const {
		return workspaceBounds;
	}

	bool safeEdge(const Agent &agent, const Edge &edge, double dt, bool checkSelfCollision = false) const {
		auto intermediateStates = State::interpolate(edge.start, edge.end, dt);
		intermediateStates.push_back(edge.end);
		intermediateStates.push_back(edge.start);

		return safeStates(agent, intermediateStates);
	}

	bool safeAbstractEdge(const Agent &agent, const AbstractEdge &edge, double dt, bool checkSelfCollision = false) const {
		return safeAbstractStates(agent, edge);
	}

	bool safeStates(const Agent &agent, const std::vector<State> &states) const {
		for (const auto &state : states) {
			if (!safeState(agent, state)) {
				return false;
			}
		}

		return true;
	}

	bool safeAbstractStates(const Agent &agent, const std::vector<AbstractState> &states) const {
		for (const auto &state : states) {
			if (!safeAbstractState(agent, state)) {
				return false;
			}
		}

		return true;
	}

	bool safeState(const Agent &agent, const State &state) const {
		const auto &stateVars = state.getTreeStateVars();

		for (auto var : stateVars) {
			if (var < 0 || var > 1) {
				return false;
			}
		}

		for(const auto &rect : obstacles) {
			if(rect.contains(state.getStateVars())) {
				return false;
			}
		}

		return true;
	}

	bool safeAbstractState(const Agent &agent, const AbstractState &state) const {
		const auto &stateVars = state.getTreeStateVars();

		for (auto var : stateVars) {
			if (var < 0 || var > 1) {
				return false;
			}
		}

		for(const auto &rect : obstacles) {
			if(rect.contains(state.getStateVars())) {
				return false;
			}
		}

		return true;
	}

#ifdef WITHGRAPHICS

	void draw() const {
		for(const auto &rect : obstacles) {
			rect.draw();
		}
		drawBoundingBox();
	}

	void drawBoundingBox() const {
		OpenGLWrapper::getOpenGLWrapper().drawLine(0,0,0,0,1,0,OpenGLWrapper::Color::Blue());
		OpenGLWrapper::getOpenGLWrapper().drawLine(0,1,0,1,1,0,OpenGLWrapper::Color::Blue());
		OpenGLWrapper::getOpenGLWrapper().drawLine(1,1,0,1,0,0,OpenGLWrapper::Color::Blue());
		OpenGLWrapper::getOpenGLWrapper().drawLine(1,0,0,0,0,0,OpenGLWrapper::Color::Blue());
	}

#endif

private:
	WorkspaceBounds workspaceBounds;
	std::vector<Rectangle> obstacles;
};
