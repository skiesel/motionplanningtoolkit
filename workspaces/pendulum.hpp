#pragma once

class Pendulum {
	typedef std::vector<double> StateVars;

	struct State {
		State() : stateVars(2, 0) {}

#ifdef WITHGRAPHICS
		void draw() const {
			double normTheta = stateVars[0] - M_PI / 2;
			double x = cos(normTheta);
			double y = sin(normTheta);

			OpenGLWrapper::getOpenGLWrapper().drawLine(0,0,0,x,y,0,OpenGLWrapper::Color::Blue());
		}
#endif
		StateVars stateVars;
	};
public:

	Pendulum(const InstanceFileMap &args) {
		controls = {-2, 0, 2};
	}

	State randomSteer(const State& state, double dt) {
		State newState(state);

		double randomControl = controls[(unsigned int)(zeroToOne(GlobalRandomGenerator) * controls.size())];

		doStep(newState, randomControl, dt);

		return newState;
	}

	bool isGoal(const State &state, const State &goal) const {
		// within 10 degrees of verticle and absolute angular velocity less than 0.5 rad/sec
		return false;
	}

#ifdef WITHGRAPHICS
	void draw() {
		state.draw();
		state = randomSteer(state, 0.5);
	}
#endif

private:
	void doStep(State& state, double torque, double dt) {
		double stepSize = 0.01;
		unsigned int steps = dt / stepSize;
		for(unsigned int i = 0; i < steps; ++i) {
			double theta = state.stateVars[0];
			double omega = state.stateVars[1];
			state.stateVars[0] += omega * stepSize;
			state.stateVars[1] += (-9.8 * sin(theta) + torque) * stepSize;
		}
	}


	State state;
	std::vector<double> controls;
	std::uniform_real_distribution<double> zeroToOne;
};