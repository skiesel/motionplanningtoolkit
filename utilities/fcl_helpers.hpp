#pragma once

#include <fcl/collision.h>
#include <fcl/collision_data.h>
#include <fcl/collision_object.h>
#include <fcl/continuous_collision.h>
#include <fcl/math/transform.h>
#include <fcl/distance.h>

#include <boost/tokenizer.hpp>

using namespace fcl;

namespace fcl_helpers {

Transform3f parseTransform(const std::string transform) {
	boost::char_separator<char> sep(" ");
	boost::tokenizer< boost::char_separator<char> > tokens(transform, sep);
	auto token = tokens.begin();

	Vec3f vector(std::stod(*token), std::stod(*++token), std::stod(*++token));
	Quaternion3f quaternion(std::stod(*++token), std::stod(*++token), std::stod(*++token), std::stod(*++token));

	return Transform3f(quaternion, vector);
}

std::vector<double> fclTransformToOpenGL(const Transform3f &transform) {
	std::vector<double> vec(16);

	const fcl::Vec3f &transVec = transform.getTranslation();
	const fcl::Matrix3f &rot = transform.getRotation();

	std::vector<double> transMatrix(16, 0);
	std::vector<double> rotationMatrix(16, 0);

	for(unsigned int i = 0; i < 16; i++) {
		if(i % 4 == i / 4) {
			transMatrix[i] = 1;
			rotationMatrix[i] = 1;
		}
	}

	transMatrix[3] = transVec[0];
	transMatrix[7] = transVec[1];
	transMatrix[11] = transVec[2];

	for(unsigned int i = 0; i < 3; i++) {

		const fcl::Vec3f &row = rot.getRow(i);
		for(unsigned int j = 0; j < 3; j++) {
			//yes, this should be 4
			rotationMatrix[i * 4 + j] = row[j];
		}
	}

	math::multiply(rotationMatrix, transMatrix, vec);

	return vec;
}

struct CollisionData {
	CollisionData() : done(false) {}

	CollisionRequest request;
	CollisionResult result;
	bool done;
};

struct DistanceData {
	DistanceData() : done(false) {}

	DistanceRequest request;
	DistanceResult result;
	bool done;

};

struct ContinuousCollisionData {
	ContinuousCollisionData() : done(false) {}

	ContinuousCollisionRequest request;
	ContinuousCollisionResult result;
	bool done;
};

bool defaultCollisionFunction(CollisionObject *o1, CollisionObject *o2, void *cdata_) {
	CollisionData *cdata = static_cast<CollisionData *>(cdata_);
	const CollisionRequest &request = cdata->request;
	CollisionResult &result = cdata->result;

	if(cdata->done) return true;

	collide(o1, o2, request, result);

	if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
		cdata->done = true;

	return cdata->done;
}

bool defaultDistanceFunction(CollisionObject *o1, CollisionObject *o2, void *cdata_, FCL_REAL &dist) {
	DistanceData *cdata = static_cast<DistanceData *>(cdata_);
	const DistanceRequest &request = cdata->request;
	DistanceResult &result = cdata->result;

	if(cdata->done) {
		dist = result.min_distance;
		return true;
	}

	distance(o1, o2, request, result);

	dist = result.min_distance;

	if(dist <= 0) return true; // in collision or in touch

	return cdata->done;
}

bool defaultContinuousCollisionFunction(ContinuousCollisionObject *o1, ContinuousCollisionObject *o2, void *cdata_) {
	ContinuousCollisionData *cdata = static_cast<ContinuousCollisionData *>(cdata_);
	const ContinuousCollisionRequest &request = cdata->request;
	ContinuousCollisionResult &result = cdata->result;

	if(cdata->done) return true;

	collide(o1, o2, request, result);

	return cdata->done;
}

void loadOBJFile(const char *filename, std::vector<Vec3f> &points, std::vector<Triangle> &triangles) {

	FILE *file = fopen(filename, "rb");
	if(!file) {
		std::cerr << "file does not exist" << std::endl;
		return;
	}

	bool has_normal = false;
	bool has_texture = false;
	char line_buffer[2000];
	while(fgets(line_buffer, 2000, file)) {
		char *first_token = strtok(line_buffer, "\r\n\t ");
		if(!first_token || first_token[0] == '#' || first_token[0] == 0)
			continue;

		switch(first_token[0]) {
		case 'v': {
			if(first_token[1] == 'n') {
				strtok(NULL, "\t ");
				strtok(NULL, "\t ");
				strtok(NULL, "\t ");
				has_normal = true;
			} else if(first_token[1] == 't') {
				strtok(NULL, "\t ");
				strtok(NULL, "\t ");
				has_texture = true;
			} else {
				FCL_REAL x = (FCL_REAL)atof(strtok(NULL, "\t "));
				FCL_REAL y = (FCL_REAL)atof(strtok(NULL, "\t "));
				FCL_REAL z = (FCL_REAL)atof(strtok(NULL, "\t "));
				Vec3f p(x, y, z);
				points.push_back(p);
			}
		}
		break;
		case 'f': {
			Triangle tri;
			char *data[30];
			int n = 0;
			while((data[n] = strtok(NULL, "\t \r\n")) != NULL) {
				if(strlen(data[n]))
					n++;
			}

			for(int t = 0; t < (n - 2); ++t) {
				if((!has_texture) && (!has_normal)) {
					tri[0] = atoi(data[0]) - 1;
					tri[1] = atoi(data[1]) - 1;
					tri[2] = atoi(data[2]) - 1;
				} else {
					const char *v1;
					for(int i = 0; i < 3; i++) {
						// vertex ID
						if(i == 0)
							v1 = data[0];
						else
							v1 = data[t + i];

						tri[i] = atoi(v1) - 1;
					}
				}
				triangles.push_back(tri);
			}
		}
		}
	}
}

};