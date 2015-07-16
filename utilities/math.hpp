#pragma once

#include <random>
#include <fcl/math/transform.h>

namespace math {
	std::uniform_real_distribution<double> zeroToOne;

	void multiply(const std::vector<double> &m1, const std::vector<double> &m2, std::vector<double> &out) {
		std::vector<double> temp(16);
		for(unsigned int row = 0; row < 4; ++row) {
			for(unsigned int col = 0; col < 4; ++col) {
				double sum = 0;
				for(unsigned int i = 0; i < 4; i++) {
					double elem1 = m1[row * 4 + i];
					double elem2 = m2[col + 4 * i];
					sum += elem1 * elem2;
				}
				temp[row * 4 + col] = sum;
			}
		}
		for(unsigned int i = 0; i < 16; i++) out[i] = temp[i];
	}


	fcl::Vec3f randomPointInSphere(double maxRadius = 1) {
		double radius = maxRadius * pow(zeroToOne(GlobalRandomGenerator), 1/3);
		double theta = 2 * M_PI * zeroToOne(GlobalRandomGenerator);
		double phi = acos(2 * zeroToOne(GlobalRandomGenerator) - 1);
		double u = cos(phi);

		double t1 = sqrt(1 - u * u);

		return fcl::Vec3f(t1 * cos(theta) * radius, t1 * sin(theta) * radius, u * radius);
	}

	double vectorDistance(const fcl::Vec3f &v1, const fcl::Vec3f &v2) {
		fcl::Vec3f diff = v1 - v2;
		return sqrt(diff.dot(diff));
	}

	fcl::Quaternion3f normalize(const fcl::Quaternion3f &q) {
		double w = q.getW();
		double x = q.getX();
		double y = q.getY();
		double z = q.getZ();
		double length = sqrt(w * w + x * x + y * y + z * z);

		return fcl::Quaternion3f(w/length, x/length, y/length, z/length);
	}

	// This came from: http://www.sonycsl.co.jp/person/nielsen/visualcomputing/programs/slerp.cpp
	fcl::Quaternion3f slerp(fcl::Quaternion3f q1, fcl::Quaternion3f q2, double lambda) {
		float dotproduct = q1.dot(q2);

		if(dotproduct >= 1) {
			return q1;
		}

		// algorithm adapted from Shoemake's paper
		double lamdaHalf = lambda / 2.0;

		double theta = acos(dotproduct);
		if(theta < 0.0) {
			theta = -theta;
		}

		double st = sin(theta);
		double sut = sin(lamdaHalf * theta);
		double sout = sin((1 - lamdaHalf) * theta);
		double coeff1 = sout/st;
		double coeff2 = sut/st;

		return normalize(q1 * coeff1 + q2 * coeff2);
	}

	fcl::Transform3f interpolateSingle(const fcl::Transform3f &t1, const fcl::Transform3f &t2, double percentage) {
		assert(percentage < 1);

		const fcl::Vec3f &v1 = t1.getTranslation();
		const fcl::Vec3f &v2 = t2.getTranslation();

		fcl::Vec3f position = v1 + (v2 - v1) * percentage;

		fcl::Quaternion3f quaternion = slerp(t1.getQuatRotation(), t2.getQuatRotation(), percentage);

		return fcl::Transform3f(quaternion, position);
	}

	std::vector<fcl::Transform3f> interpolate(const fcl::Transform3f &t1, const fcl::Transform3f &t2, double linearStepSize) {
		std::vector<fcl::Transform3f> interpolationPoints;

		const fcl::Vec3f &v1 = t1.getTranslation();
		const fcl::Vec3f &v2 = t2.getTranslation();

		double totalLinearDistance = vectorDistance(v1, v2);

		if(linearStepSize > totalLinearDistance) {
			interpolationPoints.emplace_back(t1);
			interpolationPoints.emplace_back(t2);
			return interpolationPoints;
		}

		unsigned int steps = totalLinearDistance / linearStepSize;
		double percentageStep = linearStepSize / totalLinearDistance;

		fcl::Vec3f vecStep = (v2 - v1) / steps;

		const fcl::Quaternion3f &q1 = t1.getQuatRotation();
		const fcl::Quaternion3f &q2 = t2.getQuatRotation();


		interpolationPoints.emplace_back(t1);

		fcl::Transform3f point(t1);
		for(unsigned int i = 0; i < steps; ++i) {
			fcl::Quaternion3f slerped = slerp(q1, q2, percentageStep * (double)(i+1));
			point.setTransform(slerped, point.getTranslation() + vecStep);
			interpolationPoints.emplace_back(point);
		}

		interpolationPoints.emplace_back(t2);

		return interpolationPoints;
	}

	fcl::Quaternion3f getRandomZOnlyQuaternion() {
		double rad = zeroToOne(GlobalRandomGenerator) * 2 * M_PI;
		fcl::Quaternion3f quaternion;
		fcl::Vec3f axis(0,0,1);
		quaternion.fromAxisAngle(axis, rad);
		return quaternion;
	}

	// From OMPL random.cpp
	// From: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III,
	fcl::Quaternion3f getRandomUnitQuaternion() {
		double x0 = zeroToOne(GlobalRandomGenerator);
		double r1 = sqrt(1.0 - x0);
		double r2 = sqrt(x0);
		double t1 = 2.0 * M_PI * zeroToOne(GlobalRandomGenerator);
		double t2 = 2.0 * M_PI * zeroToOne(GlobalRandomGenerator);
		double c1 = cos(t1), s1 = sin(t1);
		double c2 = cos(t2), s2 = sin(t2);

		return fcl::Quaternion3f(s1 * r1,
								 c1 * r1,
								 s2 * r2,
								 c2 * r2);
	}
}