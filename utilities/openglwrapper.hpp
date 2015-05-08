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
#include <fstream>
#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

#include "connexion_3d_mouse.hpp"

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
	const GLchar *vertexSource = "../utilities/shaders/shader.vert";
	const GLchar *fragmentSource = "../utilities/shaders/shader.frag";

public:

	class Color {
	public:
		Color(double r = 1, double g = 1, double b = 1, double a = 1) : color(4) {
			color[0] = r;
			color[1] = g;
			color[2] = b;
			color[3] = a;
		}
		const std::vector<double>& getColor() const { return color; }
		static Color Red() { return Color(1,0,0); }
		static Color Green() { return Color(0,1,0); }
		static Color Blue() { return Color(0,0,1); }
	private:
		std::vector<double> color;
	};

	static OpenGLWrapper& getOpenGLWrapper() {
		if(wrapperInstance == NULL) {
			wrapperInstance = new OpenGLWrapper();
		}
		return *wrapperInstance;
	}

	static void setExternalKeyboardCallback(std::function<void(int)> keyboard) {
		externalKeyboardCallback = keyboard;
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

		glEnable(GL_CULL_FACE);
		glDepthFunc(GL_LEQUAL);
		glEnable(GL_DEPTH_TEST);

		// Create Vertex Array Object
		GLuint vao;
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);

		// Create and compile the vertex shader
		GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
		std::string src = readShader(vertexSource);
		const char *srcPtr = src.c_str();
		glShaderSource(vertexShader, 1, &srcPtr, NULL);
		glCompileShader(vertexShader);

		// Create and compile the fragment shader
		GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
		src = readShader(fragmentSource);
		srcPtr = src.c_str();
		glShaderSource(fragmentShader, 1, &srcPtr, NULL);
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

		Connexion3DMouse::createConnexion3DMouse();

		while(!glfwWindowShouldClose(window)) {
			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClearDepthf(1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			// xRot += M_PI / ((double)rand() / (double)RAND_MAX * 10. + 170);
			// yRot += M_PI / ((double)rand() / (double)RAND_MAX * 10. + 170);

			// double sinDeltaX = sin(xRot);
			// double cosDeltaX = cos(xRot);

			// double sinDeltaY = sin(yRot);
			// double cosDeltaY = cos(yRot);

			// yRotateMatrix[5] = cosDeltaY;
			// yRotateMatrix[6] = -sinDeltaY;
			// yRotateMatrix[9] = sinDeltaY;
			// yRotateMatrix[10] = cosDeltaY;

			// xRotateMatrix[0] = cosDeltaX;
			// xRotateMatrix[2] = sinDeltaX;
			// xRotateMatrix[8] = -sinDeltaX;
			// xRotateMatrix[10] = cosDeltaX;

			buildTransform();
			glUniformMatrix4fv(transformInt, 1, true, transformMatrix);

			callback();

			glfwSwapBuffers(window);
			glfwPollEvents();
			handle3DMouseEvents();
		}

		Connexion3DMouse::destroyConnexion3DMouse();

		glDeleteProgram(shaderProgram);
		glDeleteShader(fragmentShader);
		glDeleteShader(vertexShader);

		glDeleteVertexArrays(1, &vao);

		glfwTerminate();
	}

	static void handle3DMouseEvents() {
		Connexion3DMouse::MouseState state = Connexion3DMouse::getLatestState();
		OpenGLWrapper& gl = getOpenGLWrapper();
		gl.translate(0, 1, state.tx / 1000);
		gl.translate(1, 1, state.ty / 1000);
		gl.translate(2, 1, state.tz / 1000);
		gl.rotate(0, 1, state.rx / 1000);
		gl.rotate(1, 1, state.ry / 1000);
		gl.rotate(2, 1, state.rz / 1000);
	}

	static void keyboard(GLFWwindow *window, int key, int scancode, int action, int mods) {
		if(action == GLFW_PRESS) {
			externalKeyboardCallback(key);
		}

		if(action == 0) {
			switch(key) {
				case GLFW_KEY_ESCAPE:
					glfwSetWindowShouldClose(window, GL_TRUE);
					break;
				case GLFW_KEY_LEFT_SHIFT:
				case GLFW_KEY_RIGHT_SHIFT:
					getOpenGLWrapper().setShiftModifier(action);
					break;
				case '-': getOpenGLWrapper().zoom(-1); break;
				case '=': if(mods == GLFW_MOD_SHIFT) getOpenGLWrapper().zoom(1); break;
				case 'A': mods == GLFW_MOD_SHIFT ? getOpenGLWrapper().translate(0, 1) : getOpenGLWrapper().translate(0, -1); break;
				case 'S': mods == GLFW_MOD_SHIFT ? getOpenGLWrapper().translate(1, -1) : getOpenGLWrapper().translate(1, 1); break;
				case 'D': mods == GLFW_MOD_SHIFT ? getOpenGLWrapper().translate(2, 1) : getOpenGLWrapper().translate(2, -1); break;
				case 'Z': mods == GLFW_MOD_SHIFT ? getOpenGLWrapper().rotate(0, 1) : getOpenGLWrapper().rotate(0, -1); break;
				case 'X': mods == GLFW_MOD_SHIFT ? getOpenGLWrapper().rotate(1, -1) : getOpenGLWrapper().rotate(1, 1); break;
				case 'C': mods == GLFW_MOD_SHIFT ? getOpenGLWrapper().rotate(2, 1) : getOpenGLWrapper().rotate(2, -1); break;
			}
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

	void translate(int axis, double direction, double magnitude = 1) {
		double translateDistance = 0.1 * magnitude;
		if(axis == 0) {
			translateMatrix[3] += translateDistance * direction;
		} else if(axis == 1) {
			translateMatrix[7] -= translateDistance * direction;
		} else if(axis == 2) {
			translateMatrix[11] += translateDistance * direction;
		}

	}

	void zoom(double direction, double magnitude = 1) {
		double scaleFactor = (direction < 0 ? 0.9 : 1.1) * magnitude;
		scaleMatrix[0] *= scaleFactor;
		scaleMatrix[5] *= scaleFactor;
		scaleMatrix[10] *= scaleFactor;
	}

	void rotate(int axis, double direction, double magnitude = 1) {
		double rotation = 0.1745 * magnitude;

		if(axis == 0) {
			xRot += rotation * direction;
			double sinVal = sin(xRot);
			double cosVal = cos(xRot);

			xRotateMatrix[0] = cosVal;
			xRotateMatrix[2] = sinVal;
			xRotateMatrix[8] = -sinVal;
			xRotateMatrix[10] = cosVal;
		} else if(axis == 1) {
			yRot += rotation * direction;
			double sinVal = sin(yRot);
			double cosVal = cos(yRot);

			yRotateMatrix[5] = cosVal;
			yRotateMatrix[6] = -sinVal;
			yRotateMatrix[9] = sinVal;
			yRotateMatrix[10] = cosVal;
		} else if(axis == 2) {
			zRot += rotation * direction;
			double sinVal = sin(zRot);
			double cosVal = cos(zRot);

			zRotateMatrix[0] = cosVal;
			zRotateMatrix[1] = -sinVal;
			zRotateMatrix[4] = sinVal;
			zRotateMatrix[5] = cosVal;
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
						double dx = fabs(x - mouseInfo.oldX);
						double dy = fabs(y - mouseInfo.oldY);

						if(dx >= dy) 
							xRot += M_PI / 180 * ((x - mouseInfo.oldX > 0) ? 1 : -1);
						else
							yRot += M_PI / 180 * ((y - mouseInfo.oldY > 0) ? 1 : -1);

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
		auto lambda = [&](){ glDrawArrays(GL_LINE_LOOP, 0, verts.size() / 28); };
		drawCommon(lambda, verts);
	}

	void drawTriangles(const std::vector<double> &verts) const {
		auto lambda = [&](){ glDrawArrays(GL_TRIANGLE_STRIP, 0, verts.size() / 28); };
		drawCommon(lambda, verts);
	}

	void drawPoints(const std::vector<double> &verts) const {
		auto lambda = [&](){ glDrawArrays(GL_POINTS, 0, verts.size() / 28); };
		drawCommon(lambda, verts);
	}

	void drawLines(const std::vector<double> &verts) const {
		auto lambda = [&](){ glDrawArrays(GL_LINE_STRIP, 0, verts.size() / 28); };
		drawCommon(lambda, verts);
	}

	const std::vector<double>& getIdentity() const { return Identity; }

private:
	OpenGLWrapper() : xRot(0), yRot(0), zRot(0), Identity(16, 0) {
		makeIdentity(transformMatrix);
		makeIdentity(scaleMatrix);
		makeIdentity(translateMatrix);
		makeIdentity(xRotateMatrix);
		makeIdentity(yRotateMatrix);
		makeIdentity(zRotateMatrix);
		scaleMatrix[0] = 0.01;
		scaleMatrix[5] = 0.01;
		scaleMatrix[10] = 0.01;

		Identity[0] = Identity[5] = Identity[10] = Identity[15] = 1;
	}

	std::string readShader(const char* filename) const {
		std::fstream file;
  		file.open(filename, std::fstream::in);

		if(!file.is_open()) {
			fprintf(stderr, "can't open shader file: %s\n", filename);
			exit(1);
		}

		std::string shader;
		std::string line;
		while(!file.eof()) {
			std::getline(file, line);
			shader += line + "\n";
		}
		file.close();
		return shader;
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
		glVertexAttribPointer(posAttrib, 4, GL_FLOAT, GL_FALSE, 28*sizeof(float), (void*)0);

		GLint normAttrib = glGetAttribLocation(shaderProgram, "normal");
		glEnableVertexAttribArray(normAttrib);
		glVertexAttribPointer(normAttrib, 4, GL_FLOAT, GL_FALSE, 28*sizeof(float), (void*)(4*sizeof(float)));

		GLint colAttrib = glGetAttribLocation(shaderProgram, "color");
		glEnableVertexAttribArray(colAttrib);
		glVertexAttribPointer(colAttrib, 4, GL_FLOAT, GL_FALSE, 28*sizeof(float), (void*)(8*sizeof(float)));

		GLint transformAttrib = glGetAttribLocation(shaderProgram, "transform");
		glEnableVertexAttribArray(transformAttrib + 0);
		glEnableVertexAttribArray(transformAttrib + 1);
		glEnableVertexAttribArray(transformAttrib + 2);
		glEnableVertexAttribArray(transformAttrib + 3);
		glVertexAttribPointer(transformAttrib + 0, 4, GL_FLOAT, GL_FALSE, 28*sizeof(float), (void*)(12*sizeof(float)));
		glVertexAttribPointer(transformAttrib + 1, 4, GL_FLOAT, GL_FALSE, 28*sizeof(float), (void*)(16*sizeof(float)));
		glVertexAttribPointer(transformAttrib + 2, 4, GL_FLOAT, GL_FALSE, 28*sizeof(float), (void*)(20*sizeof(float)));
		glVertexAttribPointer(transformAttrib + 3, 4, GL_FLOAT, GL_FALSE, 28*sizeof(float), (void*)(24*sizeof(float)));

		drawType();

		glDeleteBuffers(1, &vbo);
	}

	void buildTransform() {
		makeIdentity(transformMatrix);

		multiply(translateMatrix, xRotateMatrix, transformMatrix);
		multiply(transformMatrix, yRotateMatrix, transformMatrix);
		multiply(transformMatrix, zRotateMatrix, transformMatrix);
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
	GLfloat zRotateMatrix[16];
	double xRot, yRot, zRot;
	GLfloat transformMatrix[16];
	MouseInfo mouseInfo;
	std::vector<double> Identity;

	static OpenGLWrapper *wrapperInstance;
	static std::function<void(int)> externalKeyboardCallback;
};

OpenGLWrapper *OpenGLWrapper::wrapperInstance = NULL;
std::function<void(int)> OpenGLWrapper::externalKeyboardCallback([](int key){});