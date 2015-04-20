#pragma once

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <fcl/collision.h>
#include <bullet/btBulletDynamicsCommon.h>

class AssimpMeshLoader {
public:
	AssimpMeshLoader(const std::string &pFile) : error(false) {
		scene = importer.ReadFile(pFile, 0);

		if(!scene) {
			fprintf(stderr, "%s\n", importer.GetErrorString());
			error = true;
		}
	}

	void get(std::vector< std::vector<fcl::Vec3f> > &vertices,
				std::vector< std::vector<fcl::Triangle> > &triangles,
				std::vector< std::vector<double> > &normals) const {

		for(unsigned int i = 0; i < scene->mNumMeshes; ++i) {
			
			auto mesh = scene->mMeshes[i];

			vertices.emplace_back();
			triangles.emplace_back();
			
			for(unsigned int j = 0; j < mesh->mNumVertices; ++j) {
				auto vertex = mesh->mVertices[j];
				vertices.back().emplace_back(vertex.x, vertex.y, vertex.z);

				normals.emplace_back(3);
				if(mesh->HasNormals()) {
					normals.back()[0] = mesh->mNormals[j][0];
					normals.back()[1] = mesh->mNormals[j][1];
					normals.back()[2] = mesh->mNormals[j][2];
				} else {
					normals.back()[0] = 1;
					normals.back()[1] = 0;
					normals.back()[2] = 0;
				}
			}

			for(unsigned int j = 0; j < mesh->mNumFaces; ++j) {
				auto face = mesh->mFaces[j];

				if(face.mNumIndices != 3) {
					fprintf(stderr, "face does not have 3 vertices: %d\n", face.mNumIndices);
					continue;
				}

				triangles.back().emplace_back(face.mIndices[0],
												face.mIndices[1],
												face.mIndices[2]);
			}
		}
	}

	void get(std::vector<btTriangleMesh*> &meshes) const {
		struct pt {
			pt(double x, double y, double z) : x(x), y(y), z(z) {}
			double x, y, z;
		};

		for(unsigned int i = 0; i < scene->mNumMeshes; ++i) {
			
			meshes.push_back(new btTriangleMesh());

			auto mesh = scene->mMeshes[i];

			for(unsigned int j = 0; j < mesh->mNumVertices; ++j) {
				auto vertex = mesh->mVertices[j];
				btVector3 vert(vertex.x, vertex.y, vertex.z);
				meshes.back()->findOrAddVertex(vert, false);
			}

			for(unsigned int j = 0; j < mesh->mNumFaces; ++j) {
				auto face = mesh->mFaces[j];

				if(face.mNumIndices != 3) {
					fprintf(stderr, "face does not have 3 vertices: %d\n", face.mNumIndices);
					continue;
				}

				meshes.back()->addTriangleIndices(face.mIndices[0],
													face.mIndices[1],
													face.mIndices[2]);
			}
		}
	}

	//don't let the importer get destructed if you want to use scene (really stupid...)
	Assimp::Importer importer;
	const aiScene *scene;
	bool error;
};