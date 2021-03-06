#pragma once

class AbstractXYZThetaTransformState {
public:
	static AbstractXYZThetaTransformState getAbstractState(const std::vector<double> &values) {
		assert(values.size() == 4);

		AbstractXYZThetaTransformState newState;

		newState.treeStateVars = values;

		fcl::Vec3f pose(values[0], values[1], values[2]);

		fcl::Vec3f zAzis(0,0,1);
		fcl::Quaternion3f quat;
		quat.fromAxisAngle(zAzis, values[3]);

		newState.transform = fcl::Transform3f(quat, pose);

		return newState;
	}

	static AbstractXYZThetaTransformState getRandomAbstractState(const std::vector< std::pair<double, double> > &bounds) {
		assert(bounds.size() == 4);

		AbstractXYZThetaTransformState newState;
		fcl::Vec3f pose;
		fcl::Quaternion3f quat;
		fcl::Vec3f zAzis(0,0,1);

		unsigned int i = 0;
		for(const auto &bound : bounds) {
			//It really feels like I'd want this to be created *once* and reused, but I am not sure
			//we can assume that the bounds will always be the same, so it should handle this generically
			std::uniform_real_distribution<double> dist(bound.first, bound.second);
			newState.treeStateVars.emplace_back(dist(GlobalRandomGenerator));

			if(i < 3) {
				pose[i++] = newState.treeStateVars.back();
			} else {
				quat.fromAxisAngle(zAzis, newState.treeStateVars.back());
			}
		}

		newState.transform = fcl::Transform3f(quat, pose);

		return newState;
	}

	static double evaluateDistance(const AbstractXYZThetaTransformState &a, const AbstractXYZThetaTransformState &b) {
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

	void draw2DAbstractEdge(const AbstractXYZThetaTransformState &state, const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {

	}
#endif

	fcl::Transform3f transform;
	std::vector<double> treeStateVars;
};