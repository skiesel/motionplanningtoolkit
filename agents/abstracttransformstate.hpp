#pragma once

class AbstractTransformState {
public:
	AbstractTransformState() {}

	const std::vector<double>& getTreeStateVars() const {
		return treeStateVars;
	}

	static AbstractTransformState getRandomAbstractState(const std::vector< std::pair<double, double> > &bounds) {
		AbstractTransformState newState;
		for(const auto &bound : bounds) {
			//It really feels like I'd want this to be created *once* and reused, but I am not sure
			//we can assume that the bounds will always be the same, so it should handle this generically
			std::uniform_real_distribution<double> dist(bound.first, bound.second);
			newState.treeStateVars.emplace_back(dist(GlobalRandomGenerator));
		}

		fcl::Vec3f pose(newState.treeStateVars[0], newState.treeStateVars[1], newState.treeStateVars[2]);

		// fcl::Matrix3f rotation;
		// rotation.setIdentity();

		// double sinVal = sin(newState.treeStateVars[3]);
		// double cosVal = cos(newState.treeStateVars[3]);

		// rotation(0,0) = cosVal;
		// rotation(1,0) = -sinVal;
		// rotation(0,1) = sinVal;
		// rotation(1,1) = cosVal;

		newState.transform = fcl::Transform3f(pose);//fcl::Transform3f(rotation, pose);

		return newState;
	}

	static double evaluateDistance(const AbstractTransformState &a, const AbstractTransformState &b) {
		assert(a.treeStateVars.size() == b.treeStateVars.size());
		double sum = 0;
		for(unsigned int i = 0; i < 3; ++i) {
			double diff = a.treeStateVars[i] - b.treeStateVars[i];
			sum += diff * diff;
		}
		return sqrt(sum);
	}

	static std::vector<AbstractTransformState> interpolate(const AbstractTransformState &a, const AbstractTransformState &b, double dt) {
		std::vector<AbstractTransformState> states;

		std::vector<double> diffs(a.treeStateVars.size());
		for(unsigned int i = 0; i < a.treeStateVars.size(); ++i) {
			diffs[i] = b.treeStateVars[i] - a.treeStateVars[i];
		}

		double dist = evaluateDistance(a, b);
		states.push_back(a);
		for(double cur = dt; cur < dist; cur+=dt) {
			states.push_back(a);
			for(unsigned int i = 0; i < a.treeStateVars.size(); ++i) {
				states.back().treeStateVars[i] += (cur / dist) * diffs[i];
			}
		}

		states.push_back(b);
		return states;
	}

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

	void draw2DAbstractEdge(const AbstractTransformState &state, const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {

	}
#endif

	fcl::Transform3f transform;
	std::vector<double> treeStateVars;
};