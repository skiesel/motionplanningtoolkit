#pragma once

class AbstractXYZTransformState {
public:
	static AbstractXYZTransformState getAbstractState(const std::vector<double> &values) {
		assert(values.size() == 3);

		AbstractXYZTransformState newState;

		newState.treeStateVars = values;

		fcl::Vec3f pose(values[0], values[1], values[2]);

		newState.transform = fcl::Transform3f(pose);

		return newState;
	}

	static AbstractXYZTransformState getRandomAbstractState(const std::vector< std::pair<double, double> > &bounds) {
		assert(bounds.size() == 3);

		AbstractXYZTransformState newState;
		fcl::Vec3f pose;
		unsigned int i = 0;
		for(const auto &bound : bounds) {
			//It really feels like I'd want this to be created *once* and reused, but I am not sure
			//we can assume that the bounds will always be the same, so it should handle this generically
			std::uniform_real_distribution<double> dist(bound.first, bound.second);
			newState.treeStateVars.emplace_back(dist(GlobalRandomGenerator));
			pose[i++] = newState.treeStateVars.back();
		}

		newState.transform = fcl::Transform3f(pose);

		return newState;
	}

	static double evaluateDistance(const AbstractXYZTransformState &a, const AbstractXYZTransformState &b) {
		assert(a.treeStateVars.size() == b.treeStateVars.size());
		double sum = 0;
		for(unsigned int i = 0; i < 3; ++i) {
			double diff = a.treeStateVars[i] - b.treeStateVars[i];
			sum += diff * diff;
		}
		return sqrt(sum);
	}

	const std::vector<double>& getTreeStateVars() const { return treeStateVars; }
	void print() const { for(auto v : treeStateVars) { fprintf(stderr, "%g ", v); } fprintf(stderr, "\n"); }

#ifdef WITHGRAPHICS
	void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			std::vector<double> pt(treeStateVars.begin(), treeStateVars.begin() + 3);
			pt.push_back(1);
			pt.push_back(0);
			pt.push_back(0);
			pt.push_back(1);
			pt.push_back(1);
			pt.insert(pt.end(), color.getColor().begin(), color.getColor().end());
			pt.insert(pt.end(), identity.begin(), identity.end());

			OpenGLWrapper::getOpenGLWrapper().drawPoints(pt);
	}

	void draw2DAbstractEdge(const AbstractXYZTransformState &state, const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {

	}
#endif

	fcl::Transform3f transform;
	std::vector<double> treeStateVars;
};