#pragma once

#ifdef WITHCONNEXION
#include <3DConnexionClient/ConnexionClientAPI.h>
#endif

#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

class Connexion3DMouse {
public:
	struct MouseState {
		MouseState() : tx(0), ty(0), tz(0), rx(0), ry(0), rz(0) {}
		double tx, ty, tz, rx, ry, rz;
	};


	static void createConnexion3DMouse() {
#ifdef WITHCONNEXION
		if(mouseThread == NULL) {
			mouseThread = new boost::thread(runloop);
		}
#endif
	}

	static void destroyConnexion3DMouse() {
#ifdef WITHCONNEXION
		if(mouseThread != NULL) {
			mouseThread->interrupt();
			//mouseThread->join();
			// delete mouseThread;
			CleanupConnexionHandlers();
		}
#endif
	}

	static MouseState getLatestState() {
#ifdef WITHCONNEXION
		boost::interprocess::scoped_lock<boost::mutex> lock(mutex);
#endif
		return mouseState;
	}

private:
	Connexion3DMouse() {}

#ifdef WITHCONNEXION
	static void added(io_connect_t connection) {}

	static void removed(io_connect_t connection) {}

	static void message(io_connect_t connection, natural_t messageType, void *messageArgument) {
		boost::interprocess::scoped_lock<boost::mutex> lock(mutex);

		switch(messageType) {
		case kConnexionMsgDeviceState: {
			ConnexionDeviceState *deviceState = (ConnexionDeviceState *) messageArgument;
			switch(deviceState->command) {
			case kConnexionCmdHandleAxis:
				//the following are intentionally swapped
				mouseState.tx =  deviceState->axis[0];
				mouseState.ty =  deviceState->axis[1];
				mouseState.tz =  deviceState->axis[2];
				mouseState.rx =  deviceState->axis[4];
				mouseState.ry =  deviceState->axis[3];
				mouseState.rz =  deviceState->axis[5];
				break;

			case kConnexionCmdHandleButtons:
				break;
			}
			break;
		}
		default:
			// fprintf(stderr, "Unknown message type\n");
			break;
		}
	}

	static void runloop() {
		if(&InstallConnexionHandlers == NULL) return;

		if(InstallConnexionHandlers(message, added, removed) != 0) return;

		RegisterConnexionClient(kConnexionClientWildcard, NULL, kConnexionClientModeTakeOver, kConnexionMaskAll);

		CFRunLoopRun();

	}

	static boost::thread *mouseThread;
	static boost::mutex mutex;
#endif

	static MouseState mouseState;
};

#ifdef WITHCONNEXION
boost::thread *Connexion3DMouse::mouseThread = NULL;
boost::mutex Connexion3DMouse::mutex;
#endif

Connexion3DMouse::MouseState Connexion3DMouse::mouseState;