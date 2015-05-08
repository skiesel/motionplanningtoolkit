#pragma once

#include "LinearMath/btIDebugDraw.h"

/*

enum  	DebugDrawModes { 
  DBG_NoDebug =0, 
  DBG_DrawWireframe = 1, 
  DBG_DrawAabb =2, 
  DBG_DrawFeaturesText =4, 
  DBG_DrawContactPoints =8, 
  DBG_NoDeactivation =16, 
  DBG_NoHelpText = 32, 
  DBG_DrawText =64, 
  DBG_ProfileTimings = 128, 
  DBG_EnableSatComparison = 256, 
  DBG_DisableBulletLCP = 512, 
  DBG_EnableCCD = 1024, 
  DBG_DrawConstraints = (1 << 11), 
  DBG_DrawConstraintLimits = (1 << 12), 
  DBG_FastWireframe = (1<<13), 
  DBG_DrawNormals = (1<<14), 
  DBG_MAX_DEBUG_DRAW_MODE 
}

*/


class GLDebugDrawer : public btIDebugDraw {
public:

	GLDebugDrawer() : m_debugMode(DBG_DrawWireframe) {}
	
	~GLDebugDrawer() {}

	void drawLine(const btVector3& from,const btVector3& to,const btVector3& fromColor, const btVector3& toColor) {
		const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

		std::vector<double> line;

		std::vector<double> pt(12);
		//point
		pt[0] = from[0];
		pt[1] = from[1];
		pt[2] = from[2];
		pt[3] = 1;

		//normal
		pt[4] = 0;
		pt[5] = 0;
		pt[6] = 1;
		pt[7] = 1;

		//color
		pt[8] = fromColor[0];
		pt[9] = fromColor[1];
		pt[10] = fromColor[2];
		pt[11] = 1;

		pt.insert(pt.end(), identity.begin(), identity.end());

		line.insert(line.end(), pt.begin(), pt.end());

		//update point
		pt[0] = to[0];
		pt[1] = to[1];
		pt[2] = to[2];

		//update color
		pt[8] = toColor[0];
		pt[9] = toColor[1];
		pt[10] = toColor[2];

		line.insert(line.end(), pt.begin(), pt.end());

		OpenGLWrapper::getOpenGLWrapper().drawLines(line);
	}

	void drawLine(const btVector3& from,const btVector3& to,const btVector3& color) {
		drawLine(from, to, color, color);
	}

	void drawSphere (const btVector3& p, btScalar radius, const btVector3& color) {
	}

	void drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha) {
		const auto &identity = OpenGLWrapper::getOpenGLWrapper().getIdentity();

		std::vector<double> triangle;

		std::vector<double> pt(12);
		//point
		pt[0] = a[0];
		pt[1] = a[1];
		pt[2] = a[2];
		pt[3] = 1;

		//normal
		pt[4] = 0;
		pt[5] = 0;
		pt[6] = 1;
		pt[7] = 1;

		//color
		pt[8] = color[0];
		pt[9] = color[1];
		pt[10] = color[2];
		pt[11] = alpha;

		pt.insert(pt.end(), identity.begin(), identity.end());

		triangle.insert(triangle.end(), pt.begin(), pt.end());

		//update point
		pt[0] = b[0];
		pt[1] = b[1];
		pt[2] = b[2];

		triangle.insert(triangle.end(), pt.begin(), pt.end());

		//update point
		pt[0] = c[0];
		pt[1] = c[1];
		pt[2] = c[2];

		triangle.insert(triangle.end(), pt.begin(), pt.end());

		OpenGLWrapper::getOpenGLWrapper().drawTriangles(triangle);
	}
	
	void drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color) {

	}

	void reportErrorWarning(const char* warningString) {

	}
	
	void draw3dText(const btVector3& location,const char* textString) {

	}

	void setDebugMode(int debugMode) {
		m_debugMode = debugMode;
	}

	int	 getDebugMode() const {
		return m_debugMode;
	}

private:
	int m_debugMode;
};



