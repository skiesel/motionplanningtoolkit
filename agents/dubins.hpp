// Copyright (c) 2008-2014, Andrew Walker
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// All the hard work is thanks to above, we just rewrapped it

#pragma once

#include <math.h>
#include <assert.h>

#include "../utilities/fcl_helpers.hpp"
#include "../utilities/openglwrapper.hpp"
#include "../utilities/instancefilemap.hpp"

#define EPSILON (1e-9)

#define UNPACK_INPUTS(alpha, beta)	\
	double sa = sin(alpha);			\
	double sb = sin(beta);			\
	double ca = cos(alpha);			\
	double cb = cos(beta);			\
	double c_ab = cos(alpha - beta);\
 
#define PACK_OUTPUTS(outputs)	\
	outputs[0]  = t;			\
	outputs[1]  = p;			\
	outputs[2]  = q;

class Dubins {
	struct DubinsPath {
		DubinsPath() {}
		DubinsPath(const DubinsPath& p) {
			for(unsigned int i = 0; i < 3; ++i) {
				qi[i] = p.qi[i];
				param[i] = p.param[i];
			}
			rho = p.rho;
			type = p.type;
		}
		double qi[3];       // the initial configuration
		double param[3];    // the lengths of the three segments
		double rho;         // model forward velocity / model angular velocity
		int type;           // path type. one of LSL, LSR, ...
	};

public:
	typedef std::vector< std::pair<double, double> > WorkspaceBounds;
	typedef std::vector< std::pair<double, double> > StateVarRanges;

	typedef std::vector<double> StateVars;

	class State {
	public:
		State() {}

		State(double x, double y, double theta) : stateVars(3) {
			stateVars[0] = x;
			stateVars[1] = y;
			stateVars[2] = theta;
		}

		State(const State &s) : stateVars(s.stateVars.begin(), s.stateVars.end()) {}

		State(const StateVars &vars) : stateVars(vars.begin(), vars.begin()+3) {}

		State& operator=(const State &s) {
			stateVars.clear();
			stateVars.insert(stateVars.begin(), s.stateVars.begin(), s.stateVars.end());
			return *this;
		}

		double x() const { return stateVars[0]; }
		double y() const { return stateVars[1]; }
		double theta() const { return stateVars[2]; }

		const bool equals(const State &s) const {
			for(unsigned int i = 0; i < 3; ++i) {
				if(fabs(stateVars[i] - s.stateVars[i]) > 0.000001) return false;
			}
			return true;
		}
		const StateVars& getStateVars() const { return stateVars; }

		void print() const {
			for(auto v : stateVars) {
				fprintf(stderr, "%g ", v);
			}
			fprintf(stderr, "\n");
		}

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			std::vector<double> pt(stateVars.begin(), stateVars.end());
			pt.push_back(1);
			pt.push_back(0);
			pt.push_back(0);
			pt.push_back(1);
			pt.push_back(1);
			pt.insert(pt.end(), color.getColor().begin(), color.getColor().end());
			pt.insert(pt.end(), identity.begin(), identity.end());

			OpenGLWrapper::getOpenGLWrapper().drawPoints(pt);
		}
#endif

	private:
		StateVars stateVars;
	};

	class Edge {
	public:
		Edge() {}

		Edge(const State &start) : start(start), end(start), cost(0), treeIndex(0) {}

		Edge(const State &start, const State &end, const DubinsPath &path, double cost) : start(start), end(end),
			path(path), cost(cost), treeIndex(0) {}

		Edge(const Edge& e) : start(e.start), end(e.end), path(e.path), cost(e.cost), treeIndex(e.treeIndex) {}

		/* needed for being inserted into NN datastructure */
		const StateVars& getTreeStateVars() const { return end.getStateVars(); }
		int getPointIndex() const { return treeIndex; }
		void setPointIndex(int ptInd) { treeIndex = ptInd; }

#ifdef WITHGRAPHICS
		void draw(const OpenGLWrapper::Color &color = OpenGLWrapper::Color()) const {
			const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

			double dt = 0.1;

			std::vector<double> lines;

			double totalLength = Edge::dubins->dubins_path_length(&path);
			double currentStep = 0;
			double vars[3];

			while(currentStep < totalLength) {
				Edge::dubins->dubins_path_sample(&path, currentStep, vars);

				lines.push_back(vars[0]);
				lines.push_back(vars[1]);
				lines.push_back(0);
				lines.push_back(1);
				lines.push_back(0);
				lines.push_back(0);
				lines.push_back(1);
				lines.push_back(1);
				lines.insert(lines.end(), color.getColor().begin(), color.getColor().end());
				lines.insert(lines.end(), identity.begin(), identity.end());

				currentStep += dt;
			}

			Edge::dubins->dubins_path_endpoint(&path, vars);
			lines.push_back(vars[0]);
			lines.push_back(vars[1]);
			lines.push_back(0);
			lines.push_back(1);
			lines.push_back(0);
			lines.push_back(0);
			lines.push_back(1);
			lines.push_back(1);
			lines.insert(lines.end(), color.getColor().begin(), color.getColor().end());
			lines.insert(lines.end(), identity.begin(), identity.end());

			OpenGLWrapper::getOpenGLWrapper().drawLines(lines);
		}
#endif

		const State start, end;
		DubinsPath path;
		double cost;
		int treeIndex;
		static const Dubins* dubins;
	};

	void updateBounds(const WorkspaceBounds &b) {
		bounds = b;
	}

	StateVarRanges getStateVarRanges(const WorkspaceBounds& b) const {
		StateVarRanges bounds(b.begin(), b.begin() + 2);
		bounds.push_back(std::make_pair(-M_PI, M_PI));
		return bounds;
	}

	unsigned int getTreeStateSize() const {
		return 3;
	}

	State buildState(const StateVars& stateVars) const {
		return State(stateVars);
	}

	bool isGoal(const State &state, const State &goal) const {
		return fabs(state.x() - goal.x()) < 0.1 &&
		fabs(state.y() - goal.y()) < 0.1 &&
		fabs(state.theta() - goal.theta()) < 0.785; //about 45 degrees
	}

	Edge steer(const State &start, const State &goal, double dt) const {
		DubinsPath path;

		dubins_init(start.getStateVars().data(), goal.getStateVars().data(), turningRadius, &path);

		double vars[3];
		double cost;
		double totalLength = dubins_path_length(&path);
		if(dt > totalLength) {
			dubins_path_endpoint(&path, vars);
			cost = totalLength;
		} else {
			dubins_path_sample(&path, dt, vars);
			DubinsPath p2;
			dubins_extract_subpath(&path, dt, &p2);
			path = p2;
			cost = dt;
		}

		return Edge(start, State(vars[0], vars[1], vars[2]), path, cost);
	}

	Edge randomSteer(const State &start, double dt) const {
		double x = ((double)rand() / (double)RAND_MAX) * (bounds[0].second - bounds[0].first) + bounds[0].second;
		double y = ((double)rand() / (double)RAND_MAX) * (bounds[1].second - bounds[1].first) + bounds[1].second;
		double theta = ((double)rand() / (double)RAND_MAX) * 2 * M_PI - M_PI;
		State randomState(x, y, theta);
		return steer(start, randomState, dt);
	}

	std::vector<const SimpleAgentMeshHandler*> getMeshes() const {
		std::vector<const SimpleAgentMeshHandler*> meshes(1, &mesh);
		return meshes;
	}

	std::vector< std::vector<fcl::Transform3f> > getRepresentivePosesForLocation(const std::vector<double> &loc) const {
		std::vector<std::vector<fcl::Transform3f> > retPoses;

		fcl::Vec3f pose(loc[0], loc[1], 0);

		unsigned int rotations = 4;
		double increment = M_PI / ((double)rotations * 2.);
		fcl::Matrix3f rotation;
		rotation.setIdentity();

		for(unsigned int i = 0; i < rotations; ++i) {
			double cosTheta = cos((double)i * increment);
			double sinTheta = sin((double)i * increment);

			rotation(0,0) = cosTheta;
			rotation(1,0) = -sinTheta;
			rotation(0,1) = sinTheta;
			rotation(1,1) = cosTheta;

			retPoses.emplace_back();
			retPoses.back().emplace_back(rotation, pose);
		}

		return retPoses;
	}

	std::vector<std::vector<fcl::Transform3f> > getPoses(const Edge &edge, double dt) const {
		std::vector<std::vector<fcl::Transform3f> > retPoses;

		double totalLength = dubins_path_length(&edge.path);
		double currentStep = 0;
		double vars[3];

		fcl::Vec3f pose;
		pose[2] = 0;
		fcl::Matrix3f rotation;
		rotation.setIdentity();

		while(currentStep < totalLength) {
			dubins_path_sample(&edge.path, currentStep, vars);

			pose[0] = vars[0];
			pose[1] = vars[1];

			double cosTheta = cos((double)vars[2]);
			double sinTheta = sin((double)vars[2]);

			rotation(0,0) = cosTheta;
			rotation(1,0) = -sinTheta;
			rotation(0,1) = sinTheta;
			rotation(1,1) = cosTheta;

			retPoses.emplace_back();
			retPoses.back().emplace_back(rotation, pose);

			currentStep += dt;
		}

		dubins_path_endpoint(&edge.path, vars);

		pose[0] = vars[0];
		pose[1] = vars[1];

		double cosTheta = cos((double)vars[2]);
		double sinTheta = sin((double)vars[2]);

		rotation(0,0) = cosTheta;
		rotation(1,0) = -sinTheta;
		rotation(0,1) = sinTheta;
		rotation(1,1) = cosTheta;

		retPoses.emplace_back();
		retPoses.back().emplace_back(rotation, pose);

		return retPoses;
	}

#ifdef WITHGRAPHICS
	void draw() const {
		mesh.draw();
	}

	void drawSolution(const std::vector<const Edge*> &solution, double dt = std::numeric_limits<double>::infinity()) const {
		for(const Edge *edge : solution) {
			std::vector< std::vector<fcl::Transform3f> > poses = getPoses(*edge, dt);
			auto transform = OpenGLWrapper::getOpenGLWrapper().getIdentity();
			for(auto pose : poses) {
				const Matrix3f &rotation = pose[0].getRotation();
				transform[0] = rotation(0,0);
				transform[1] = rotation(1,0);
				transform[2] = rotation(2,0);

				transform[4] = rotation(0,1);
				transform[5] = rotation(1,1);
				transform[6] = rotation(2,1);

				transform[8] = rotation(0,2);
				transform[9] = rotation(1,2);
				transform[10] = rotation(2,2);

				const Vec3f &translation = pose[0].getTranslation();
				transform[12] += translation[0];
				transform[13] += translation[1];
				// transform[14] += translation[2];

				mesh.draw(OpenGLWrapper::Color(), transform);
			}
		}
	}

	void animateSolution(const std::vector<const Edge*> &solution, unsigned int poseNumber) const {
		auto transform = OpenGLWrapper::getOpenGLWrapper().getIdentity();
		unsigned int edgeNumber = poseNumber / 2;
		unsigned int endpoint = poseNumber % 2;
		const Edge *edge = solution[edgeNumber];
		std::vector< std::vector<fcl::Transform3f> > poses = getPoses(*edge, std::numeric_limits<double>::infinity());
		
		const Matrix3f &rotation = poses[endpoint][0].getRotation();
		transform[0] = rotation(0,0);
		transform[1] = rotation(1,0);
		transform[2] = rotation(2,0);

		transform[4] = rotation(0,1);
		transform[5] = rotation(1,1);
		transform[6] = rotation(2,1);

		transform[8] = rotation(0,2);
		transform[9] = rotation(1,2);
		transform[10] = rotation(2,2);

		const Vec3f &translation = poses[endpoint][0].getTranslation();
		transform[12] += translation[0];
		transform[13] += translation[1];
		// transform[14] += translation[2];

		mesh.draw(OpenGLWrapper::Color(), transform);
	}
#endif

	enum { LSL = 0, LSR = 1, RSL = 2, RSR = 3, RLR = 4, LRL = 5 } PathTypes;
	enum { L_SEG = 0, S_SEG = 1, R_SEG = 2 } SegmentTypes;
	enum {
		EDUBOK = 0,			// No error
		EDUBCOCONFIGS = 1,	// Colocated configurations
		EDUBPARAM = 2,		// Path parameterisitation error
		EDUBBADRHO = 3,		// the rho value is invalid
		EDUBNOPATH = 4		// no connection between configurations with this word
	} Errors;

	Dubins(const InstanceFileMap &args) : dubins_words(6),
		mesh(args.value("Agent Mesh")), turningRadius(stod(args.value("Turning Radius"))) {
		dubins_words[0] = &dubins_LSL;
		dubins_words[1] = &dubins_LSR;
		dubins_words[2] = &dubins_RSL;
		dubins_words[3] = &dubins_RSR;
		dubins_words[4] = &dubins_RLR;
		dubins_words[5] = &dubins_LRL;

 		DIRDATA.emplace_back(3);
 		DIRDATA.back()[0] = L_SEG;
 		DIRDATA.back()[1] = S_SEG;
 		DIRDATA.back()[2] = L_SEG;

 		DIRDATA.emplace_back(3);
 		DIRDATA.back()[0] = L_SEG;
 		DIRDATA.back()[1] = S_SEG;
 		DIRDATA.back()[2] = R_SEG;

 		DIRDATA.emplace_back(3);
 		DIRDATA.back()[0] = R_SEG;
 		DIRDATA.back()[1] = S_SEG;
 		DIRDATA.back()[2] = L_SEG;

 		DIRDATA.emplace_back(3);
 		DIRDATA.back()[0] = R_SEG;
 		DIRDATA.back()[1] = S_SEG;
 		DIRDATA.back()[2] = R_SEG;

 		DIRDATA.emplace_back(3);
 		DIRDATA.back()[0] = R_SEG;
 		DIRDATA.back()[1] = L_SEG;
 		DIRDATA.back()[2] = R_SEG;

 		DIRDATA.emplace_back(3);
 		DIRDATA.back()[0] = L_SEG;
 		DIRDATA.back()[1] = R_SEG;
 		DIRDATA.back()[2] = L_SEG;

 		Edge::dubins = this;
	}

	/**
	 * Generate a path from an initial configuration to
	 * a target configuration, with a specified maximum turning
	 * radii
	 *
	 * A configuration is (x, y, theta), where theta is in radians, with zero
	 * along the line x = 0, and counter-clockwise is positive
	 *
	 * @param q0    - a configuration specified as an array of x, y, theta
	 * @param q1    - a configuration specified as an array of x, y, theta
	 * @param rho   - turning radius of the vehicle (forward velocity divided by maximum angular velocity)
	 * @param path  - the resultant path
	 * @return      - non-zero on error
	 */
	int dubins_init(const double q0[3], const double q1[3], double rho, DubinsPath *path) const {
		int i;
		double dx = q1[0] - q0[0];
		double dy = q1[1] - q0[1];
		double D = sqrt(dx * dx + dy * dy);
		double d = D / rho;
		if(rho <= 0.) {
			return EDUBBADRHO;
		}
		double theta = mod2pi(atan2(dy, dx));
		double alpha = mod2pi(q0[2] - theta);
		double beta  = mod2pi(q1[2] - theta);
		for(i = 0; i < 3; i ++) {
			path->qi[i] = q0[i];
		}
		path->rho = rho;

		return dubins_init_normalised(alpha, beta, d, path);
	}

	/**
	 * Calculate the length of an initialised path
	 *
	 * @param path - the path to find the length of
	 */
	double dubins_path_length(const DubinsPath *path) const {
		double length = 0.;
		length += path->param[0];
		length += path->param[1];
		length += path->param[2];
		length = length * path->rho;
		return length;
	}

	double dubins_segment_length(const DubinsPath *path, unsigned int segment) const {
		assert(segment < 3);
		return path->param[segment] * path->rho;
	}

	/**
	 * Extract an integer that represents which path type was used
	 *
	 * @param path    - an initialised path
	 * @return        - one of LSL, LSR, RSL, RSR, RLR or LRL (ie/ 0-5 inclusive)
	 */
	int dubins_path_type(const DubinsPath *path) const {
		return path->type;
	}

	/**
	 * Calculate the configuration along the path, using the parameter t
	 *
	 * @param path - an initialised path
	 * @param t    - a length measure, where 0 <= t < dubins_path_length(path)
	 * @param q    - the configuration result
	 * @returns    - non-zero if 't' is not in the correct range
	 */
	int dubins_path_sample(const DubinsPath *path, double t, double q[3]) const {
		if(t < 0 || t >= dubins_path_length(path)) {
			// error, parameter out of bounds
			return EDUBPARAM;
		}

		// tprime is the normalised variant of the parameter t
		double tprime = t / path->rho;

		// In order to take rho != 1 into account this function needs to be more complex
		// than it would be otherwise. The transformation is done in five stages.
		//
		// 1. translate the components of the initial configuration to the origin
		// 2. generate the target configuration
		// 3. transform the target configuration
		//      scale the target configuration
		//      translate the target configration back to the original starting point
		//      normalise the target configurations angular component

		// The translated initial configuration
		double qi[3] = { 0, 0, path->qi[2] };

		// Generate the target configuration
		const std::vector<int> &types = DIRDATA[path->type];

		double p1 = path->param[0];
		double p2 = path->param[1];
		double q1[3]; // end-of segment 1
		double q2[3]; // end-of segment 2
		dubins_segment(p1, qi, q1, types[0]);
		dubins_segment(p2, q1, q2, types[1]);
		if(tprime < p1) {
			dubins_segment(tprime, qi, q, types[0]);
		} else if(tprime < (p1+p2)) {
			dubins_segment(tprime-p1, q1, q,  types[1]);
		} else {
			dubins_segment(tprime-p1-p2, q2, q,  types[2]);
		}

		// scale the target configuration, translate back to the original starting point
		q[0] = q[0] * path->rho + path->qi[0];
		q[1] = q[1] * path->rho + path->qi[1];
		q[2] = mod2pi(q[2]);

		return 0;
	}

	/**
	 * Callback function for path sampling
	 *
	 * @note the q parameter is a configuration
	 * @note the t parameter is the distance along the path
	 * @note the user_data parameter is forwarded from the caller
	 * @note return non-zero to denote sampling should be stopped
	 */
	typedef int (*DubinsPathSamplingCallback)(double q[3], double t, void *user_data);

	/**
	 * Walk along the path at a fixed sampling interval, calling the
	 * callback function at each interval
	 *
	 * @param path      - the path to sample
	 * @param cb        - the callback function to call for each sample
	 * @param user_data - optional information to pass on to the callback
	 * @param stepSize  - the distance along the path for subsequent samples
	 */
	int dubins_path_sample_many(const DubinsPath *path, DubinsPathSamplingCallback cb, double stepSize, void *user_data) const {
		double x = 0.0;
		double length = dubins_path_length(path);
		while(x <  length) {
			double q[3];
			dubins_path_sample(path, x, q);
			int retcode = cb(q, x, user_data);
			if(retcode != 0) {
				return retcode;
			}
			x += stepSize;
		}
		return 0;
	}

	/**
	 * Convenience function to identify the endpoint of a path
	 *
	 * @param path - an initialised path
	 * @param q    - the configuration result
	 */
	int dubins_path_endpoint(const DubinsPath *path, double q[3]) const {
		// TODO - introduce a new constant rather than just using EPSILON
		return dubins_path_sample(path, dubins_path_length(path) - EPSILON, q);
	}

	/**
	 * Convenience function to extract a subset of a path
	 *
	 * @param path    - an initialised path
	 * @param t       - a length measure, where 0 < t < dubins_path_length(path)
	 * @param newpath - the resultant path
	 */
	int dubins_extract_subpath(const DubinsPath *path, double t, DubinsPath *newpath) const {
		// calculate the true parameter
		double tprime = t / path->rho;

		// copy most of the data
		newpath->qi[0] = path->qi[0];
		newpath->qi[1] = path->qi[1];
		newpath->qi[2] = path->qi[2];
		newpath->rho   = path->rho;
		newpath->type  = path->type;

		// fix the parameters
		newpath->param[0] = fmin(path->param[0], tprime);
		newpath->param[1] = fmin(path->param[1], tprime - newpath->param[0]);
		newpath->param[2] = fmin(path->param[2], tprime - newpath->param[0] - newpath->param[1]);
		return 0;
	}

private:
	/**
	 * Floating point modulus suitable for rings
	 *
	 * fmod doesn't behave correctly for angular quantities, this function does
	 */
	static double fmodr(double x, double y) {
		return x - y*floor(x/y);
	}

	static double mod2pi(double theta) {
		return fmodr(theta, 2 * M_PI);
	}

	int dubins_init_normalised(double alpha, double beta, double d, DubinsPath *path) const {
		double best_cost = INFINITY;
		int    best_word;
		int    i;

		best_word = -1;
		for(i = 0; i < 6; i++) {
			double params[3];
			int err = dubins_words[i](alpha, beta, d, params);
			if(err == EDUBOK) {
				double cost = params[0] + params[1] + params[2];
				if(cost < best_cost) {
					best_word = i;
					best_cost = cost;
					path->param[0] = params[0];
					path->param[1] = params[1];
					path->param[2] = params[2];
					path->type = i;
				}
			}
		}

		if(best_word == -1) {
			return EDUBNOPATH;
		}
		path->type = best_word;
		return EDUBOK;
	}

	void dubins_segment(double t, double qi[3], double qt[3], int type) const {
		assert(type == L_SEG || type == S_SEG || type == R_SEG);

		if(type == L_SEG) {
			qt[0] = qi[0] + sin(qi[2]+t) - sin(qi[2]);
			qt[1] = qi[1] - cos(qi[2]+t) + cos(qi[2]);
			qt[2] = qi[2] + t;
		} else if(type == R_SEG) {
			qt[0] = qi[0] - sin(qi[2]-t) + sin(qi[2]);
			qt[1] = qi[1] + cos(qi[2]-t) - cos(qi[2]);
			qt[2] = qi[2] - t;
		} else if(type == S_SEG) {
			qt[0] = qi[0] + cos(qi[2]) * t;
			qt[1] = qi[1] + sin(qi[2]) * t;
			qt[2] = qi[2];
		}
	}

	static int dubins_LSL(double alpha, double beta, double d, double *outputs) {
		UNPACK_INPUTS(alpha, beta);
		double tmp0 = d+sa-sb;
		double p_squared = 2 + (d*d) -(2*c_ab) + (2*d*(sa - sb));
		if(p_squared < 0) {
			return EDUBNOPATH;
		}
		double tmp1 = atan2((cb-ca), tmp0);
		double t = mod2pi(-alpha + tmp1);
		double p = sqrt(p_squared);
		double q = mod2pi(beta - tmp1);
		PACK_OUTPUTS(outputs);
		return EDUBOK;
	}

	static int dubins_RSR(double alpha, double beta, double d, double *outputs) {
		UNPACK_INPUTS(alpha, beta);
		double tmp0 = d-sa+sb;
		double p_squared = 2 + (d*d) -(2*c_ab) + (2*d*(sb-sa));
		if(p_squared < 0) {
			return EDUBNOPATH;
		}
		double tmp1 = atan2((ca-cb), tmp0);
		double t = mod2pi(alpha - tmp1);
		double p = sqrt(p_squared);
		double q = mod2pi(-beta + tmp1);
		PACK_OUTPUTS(outputs);
		return EDUBOK;
	}

	static int dubins_LSR(double alpha, double beta, double d, double *outputs) {
		UNPACK_INPUTS(alpha, beta);
		double p_squared = -2 + (d*d) + (2*c_ab) + (2*d*(sa+sb));
		if(p_squared < 0) {
			return EDUBNOPATH;
		}
		double p    = sqrt(p_squared);
		double tmp2 = atan2((-ca-cb), (d+sa+sb)) - atan2(-2.0, p);
		double t    = mod2pi(-alpha + tmp2);
		double q    = mod2pi(-mod2pi(beta) + tmp2);
		PACK_OUTPUTS(outputs);
		return EDUBOK;
	}

	static int dubins_RSL(double alpha, double beta, double d, double *outputs) {
		UNPACK_INPUTS(alpha, beta);
		double p_squared = (d*d) -2 + (2*c_ab) - (2*d*(sa+sb));
		if(p_squared< 0) {
			return EDUBNOPATH;
		}
		double p    = sqrt(p_squared);
		double tmp2 = atan2((ca+cb), (d-sa-sb)) - atan2(2.0, p);
		double t    = mod2pi(alpha - tmp2);
		double q    = mod2pi(beta - tmp2);
		PACK_OUTPUTS(outputs);
		return EDUBOK;
	}

	static int dubins_RLR(double alpha, double beta, double d, double *outputs) {
		UNPACK_INPUTS(alpha, beta);
		double tmp_rlr = (6. - d*d + 2*c_ab + 2*d*(sa-sb)) / 8.;
		if(fabs(tmp_rlr) > 1) {
			return EDUBNOPATH;
		}
		double p = mod2pi(2*M_PI - acos(tmp_rlr));
		double t = mod2pi(alpha - atan2(ca-cb, d-sa+sb) + mod2pi(p/2.));
		double q = mod2pi(alpha - beta - t + mod2pi(p));
		PACK_OUTPUTS(outputs);
		return EDUBOK;
	}

	static int dubins_LRL(double alpha, double beta, double d, double *outputs) {
		UNPACK_INPUTS(alpha, beta);
		double tmp_lrl = (6. - d*d + 2*c_ab + 2*d*(- sa + sb)) / 8.;
		if(fabs(tmp_lrl) > 1) {
			return EDUBNOPATH;
		}
		double p = mod2pi(2*M_PI - acos(tmp_lrl));
		double t = mod2pi(-alpha - atan2(ca-cb, d+sa-sb) + p/2.);
		double q = mod2pi(mod2pi(beta) - alpha -t + mod2pi(p));
		PACK_OUTPUTS(outputs);
		return EDUBOK;
	}

	// Iteratable set of functions for creating path types
	std::vector< std::function<int(double,double,double,double*)> > dubins_words;

	// The segment types for each of the Path types
	std::vector< std::vector<int> > DIRDATA;

	SimpleAgentMeshHandler mesh;

	double turningRadius;

	WorkspaceBounds bounds;
};

const Dubins* Dubins::Edge::dubins = NULL;