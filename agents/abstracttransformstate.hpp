#pragma once

class AbstractTransformState {
public:
	AbstractTransformState() {}
	AbstractTransformState(const AbstractTransformState &s) {}

	std::vector<fcl::Transform3f> getTransforms() const {
		return std::vector<fcl::Transform3f>();
	}

	static std::vector<fcl::Transform3f> interpolate(const AbstractTransformState &a, const AbstractTransformState &b, double dt) {
		return std::vector<fcl::Transform3f>();
	}

	static AbstractTransformState getRandomAbstractState(const std::vector< std::pair<double, double> > &bounds) {
		return AbstractTransformState();
	}

	static double evaluateDistance(const AbstractTransformState &a, const AbstractTransformState &b) {
		return 0;
	}

	fcl::Transform3f transform;
	std::vector<double> treeStateVars;
};