#pragma once

#define GLEW_STATIC
#include <GL/glew.h>

#ifndef __APPLE__
#include <GL/glx.h>
#endif

#include <GLFW/glfw3.h>

#include <cmath>
#include <vector>
#include <functional>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

class OpenGLWrapper {

	// Specify prototype of function
	typedef void (*GENBUFFERS)(GLsizei, GLuint *);

	//windows?
	//GENBUFFERS glGenBuffers = (GENBUFFERS)wglGetProcAddress("glGenBuffers");
#if __APPLE__
	//GENBUFFERS glGenBuffers = (GENBUFFERS)glfwGetProcAddress("glGenBuffers");
#else
	GENBUFFERS glGenBuffers = (GENBUFFERS)glXGetProcAddress((const GLubyte *) "glGenBuffers");
#endif


	// Shader sources
	const GLchar *vertexSource =
	    "#version 150\n"
	    "in vec4 position;"
		"uniform mat4 transformMatrix;"
	    "void main() {"
	    "	gl_Position = transformMatrix * position;"
	    "}";
	const GLchar *fragmentSource =
	    "#version 150 core\n"
	    "out vec4 outColor;"
	    "void main() {"
	    "	outColor = vec4(1.0, 1.0, 1.0, 1.0);"
	    "}";

public:

	static OpenGLWrapper& getOpenGLWrapper() {
		if(wrapperInstance == NULL) {
			wrapperInstance = new OpenGLWrapper();
		}
		return *wrapperInstance;
	}

	void runWithCallback(std::function<void(void)> callback) {
		glfwInit();

		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

		glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

		GLFWwindow *window = glfwCreateWindow(800, 800, "Planner", nullptr, nullptr); // Windowed
		//	GLFWwindow* window = glfwCreateWindow(800, 600, "OpenGL", glfwGetPrimaryMonitor(), nullptr); // Fullscreen

		glfwMakeContextCurrent(window);

		glewExperimental = GL_TRUE;
		glewInit();

		// Create Vertex Array Object
		GLuint vao;
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);

		// Create and compile the vertex shader
		GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(vertexShader, 1, &vertexSource, NULL);
		glCompileShader(vertexShader);

		// Create and compile the fragment shader
		GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(fragmentShader, 1, &fragmentSource, NULL);
		glCompileShader(fragmentShader);

		// Link the vertex and fragment shader into a shader program
		shaderProgram = glCreateProgram();
		glAttachShader(shaderProgram, vertexShader);
		glAttachShader(shaderProgram, fragmentShader);
		glBindFragDataLocation(shaderProgram, 0, "outColor");
		glLinkProgram(shaderProgram);
		glUseProgram(shaderProgram);

		glfwSetKeyCallback(window, keyboard);
		glfwSetMouseButtonCallback(window, mouseClick);
		glfwSetCursorPosCallback(window, mouseMove);

		GLint transformInt = glGetUniformLocation(shaderProgram, "transformMatrix");

		while(!glfwWindowShouldClose(window)) {
			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT);

			buildTransform();
			glUniformMatrix4fv(transformInt, 1, true, transformMatrix);

			callback();

			glfwSwapBuffers(window);
			glfwPollEvents();
		}


		glDeleteProgram(shaderProgram);
		glDeleteShader(fragmentShader);
		glDeleteShader(vertexShader);

		glDeleteVertexArrays(1, &vao);

		glfwTerminate();
	}

	static void keyboard(GLFWwindow *window, int key, int scancode, int action, int mods) {
		switch(key) {
			case GLFW_KEY_ESCAPE:
				glfwSetWindowShouldClose(window, GL_TRUE);
				break;
			case GLFW_KEY_LEFT_SHIFT:
			case GLFW_KEY_RIGHT_SHIFT:
				getOpenGLWrapper().setShiftModifier(action);
				break;
		}
	}

	static void mouseClick(GLFWwindow *window, int button, int action, int mods) {
		getOpenGLWrapper().setMouseActivity(button, action);
	}

	static void mouseMove(GLFWwindow *window, double x, double y) {
		OpenGLWrapper &wrapper = getOpenGLWrapper();
		if(wrapper.isMouseClicked()) {
			wrapper.setMousePosition(x, y);
		}
	}

	bool isMouseClicked() const {
		return mouseInfo.isClicked();
	}

	void setMouseActivity(int button, int action) {
		mouseInfo.mouseButton = button;
		mouseInfo.mouseState = action;
		if(mouseInfo.mouseState == GLFW_RELEASE) {
			mouseInfo.oldCoordIsGood = false;	
		}
	}

	struct MouseInfo {
		MouseInfo() : mouseButton(-1), mouseState(-1), shiftPressed(false), oldCoordIsGood(false) {}
		bool isClicked() const {
			return mouseState == GLFW_PRESS;
		}
		bool isLeftButton() const {
			return mouseButton == GLFW_MOUSE_BUTTON_LEFT;
		}
		int mouseButton, mouseState;
		double oldX, oldY;
		bool shiftPressed, oldCoordIsGood;
	};

	void setMousePosition(double x, double y) {
		const double softenMouse = 0.001;
		if(mouseInfo.isClicked()) {
			bool leftButton = mouseInfo.isLeftButton();

			if(mouseInfo.shiftPressed && leftButton) {
				if(mouseInfo.oldCoordIsGood) {
					double scaleFactor = (y < mouseInfo.oldY) ? 0.9 : 1.1;
					scaleMatrix[0] *= scaleFactor;
					scaleMatrix[5] *= scaleFactor;
					scaleMatrix[10] *= scaleFactor;
				}
			}
			else if (!mouseInfo.shiftPressed) {
				if(leftButton) {
					if(mouseInfo.oldCoordIsGood) {
						translateMatrix[3] += (x - mouseInfo.oldX) * softenMouse;
						translateMatrix[7] -= (y - mouseInfo.oldY) * softenMouse;
					}
				}
				else {
					if(mouseInfo.oldCoordIsGood) {
						xRot += M_PI / 180 * (x - mouseInfo.oldX) ? 1 : -1 ;
						yRot += M_PI / 180 * (y - mouseInfo.oldY) ? 1 : -1 ;

						double sinDeltaX = sin(xRot);
						double cosDeltaX = cos(xRot);

						double sinDeltaY = sin(yRot);
						double cosDeltaY = cos(yRot);

						yRotateMatrix[5] = cosDeltaY;
						yRotateMatrix[6] = -sinDeltaY;
						yRotateMatrix[9] = sinDeltaY;
						yRotateMatrix[10] = cosDeltaY;

						xRotateMatrix[0] = cosDeltaX;
						xRotateMatrix[2] = sinDeltaX;
						xRotateMatrix[8] = -sinDeltaX;
						xRotateMatrix[10] = cosDeltaX;
					}
				}
			}

			mouseInfo.oldX = x;
			mouseInfo.oldY = y;
			mouseInfo.oldCoordIsGood = true;
		}
	}

	void setShiftModifier(int action) {
		mouseInfo.shiftPressed = action == GLFW_PRESS;
	}

	void drawPolygon(const std::vector<double> &verts) const {
		auto lambda = [&](){ glDrawArrays(GL_LINE_LOOP, 0, verts.size() / 4); };
		drawCommon(lambda, verts);
	}

	void drawTriangles(const std::vector<double> &verts) const {
		auto lambda = [&](){ glDrawArrays(GL_TRIANGLE_STRIP, 0, verts.size() / 4); };
		drawCommon(lambda, verts);
	}

	void drawPoints(const std::vector<double> &verts) const {
		auto lambda = [&](){ glDrawArrays(GL_POINTS, 0, verts.size() / 4); };
		drawCommon(lambda, verts);
	}

	void drawLines(const std::vector<double> &verts) const {
		auto lambda = [&](){ glDrawArrays(GL_LINE_STRIP, 0, verts.size() / 4); };
		drawCommon(lambda, verts);
	}

private:
	OpenGLWrapper() : xRot(0), yRot(0) {
		makeIdentity(transformMatrix);
		makeIdentity(scaleMatrix);
		makeIdentity(translateMatrix);
		makeIdentity(xRotateMatrix);
		makeIdentity(yRotateMatrix);
		scaleMatrix[0] = .1;
		scaleMatrix[5] = .1;
		scaleMatrix[10] = .1;
	}

	void drawCommon(std::function<void(void)> drawType, const std::vector<double> &verts) const {
		GLuint vbo;
		glGenBuffers(1, &vbo);

		GLfloat vertices[verts.size()];
		unsigned int i = 0;
		for(auto val : verts) {
			vertices[i++] = val;
		}
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		// Specify the layout of the vertex data
		GLint posAttrib = glGetAttribLocation(shaderProgram, "position");
		glEnableVertexAttribArray(posAttrib);
		glVertexAttribPointer(posAttrib, 4, GL_FLOAT, GL_FALSE, 0, (void*)0);

		drawType();

		glDeleteBuffers(1, &vbo);
	}

	void buildTransform() {
		makeIdentity(transformMatrix);

		multiply(translateMatrix, xRotateMatrix, transformMatrix);
		multiply(transformMatrix, yRotateMatrix, transformMatrix);
		multiply(transformMatrix, scaleMatrix, transformMatrix);
	}

	void makeIdentity(GLfloat* m) const {
		for(unsigned int i = 0; i < 16; i++) {
			m[i] = 0;
		}
		m[0] = 1;
		m[5] = 1;
		m[10] = 1;
		m[15] = 1;
	}

	void multiply(const GLfloat* m1, const GLfloat* m2, GLfloat* out) const {
		GLfloat temp[16];
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

	void printMatrix(const GLfloat* m) {
		for(unsigned int i = 0; i < 16; i++) {
			if(i % 4 == 0) {
				fprintf(stderr, "\n%g\t", m[i]);
			} else {
				fprintf(stderr, "%g\t", m[i]);
			}
			
		}
		fprintf(stderr, "\n");
	}


	GLuint shaderProgram;
	GLfloat scaleMatrix[16];
	GLfloat translateMatrix[16];
	GLfloat xRotateMatrix[16];
	GLfloat yRotateMatrix[16];
	double xRot, yRot;
	GLfloat transformMatrix[16];
	MouseInfo mouseInfo;

	static OpenGLWrapper *wrapperInstance;
};

OpenGLWrapper *OpenGLWrapper::wrapperInstance = NULL;