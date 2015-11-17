#pragma once

class AbstractXYThetaTransformState {
public:
	static AbstractXYThetaTransformState getRandomAbstractState(const std::vector< std::pair<double, double> > &bounds) {
		assert(bounds.size() == 3);

		AbstractXYThetaTransformState newState;
		fcl::Vec3f pose;
		fcl::Quaternion3f quat;
		fcl::Vec3f zAzis(0,0,1);

		unsigned int i = 0;
		for(const auto &bound : bounds) {
			//It really feels like I'd want this to be created *once* and reused, but I am not sure
			//we can assume that the bounds will always be the same, so it should handle this generically
			std::uniform_real_distribution<double> dist(bound.first, bound.second);
			newState.treeStateVars.emplace_back(dist(GlobalRandomGenerator));

			if(i < 2) {
				pose[i++] = newState.treeStateVars.back();
			} else {
				quat.fromAxisAngle(zAzis, newState.treeStateVars.back());
			}
		}

		newState.transform = fcl::Transform3f(quat, pose);

		return newState;
	}

	static double evaluateDistance(const AbstractXYThetaTransformState &a, const AbstractXYThetaTransformState &b) {
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

	void draw2DAbstractEdge(const AbstractXYThetaTransformState &state, const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {

	}
#endif

	fcl::Transform3f transform;
	std::vector<double> treeStateVars;
};