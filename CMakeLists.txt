cmake_minimum_required (VERSION 2.6)
project (MotionPlanning)

set(CMAKE_MODULE_PATH
  ${PROJECT_SOURCE_DIR}/CMakeModules
)
set(BUILDGRAPHICS 1)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_executable(Planner main.cpp)

if(IS_DIRECTORY "/Library/Frameworks/3DconnexionClient.framework")
	add_definitions(-DWITHCONNEXION)
	set(CONNEXION_LIBRARIES "/Library/Frameworks/3DconnexionClient.framework")
endif(IS_DIRECTORY "/Library/Frameworks/3DconnexionClient.framework")


find_package(PkgConfig REQUIRED)
pkg_search_module(FCL REQUIRED fcl)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.57.0 COMPONENTS thread system REQUIRED)

find_package(ASSIMP REQUIRED)
find_package(BULLET REQUIRED)

include_directories(
	${Boost_INCLUDE_DIRS}
	${Boost_LIBRARY_DIRS}
	${FCL_LIBRARY_DIRS}
	${ASSIMP_LIBRARY_DIRS}
	${BULLET_LIBRARY_DIRS}
	"/usr/local/include/bullet"
	"dependencies/V-REP_PRO_EDU_V3_2_1_Mac/programming/include"
)

target_link_libraries(Planner
	${Boost_LIBRARIES}
	${FCL_LIBRARIES}
	${ASSIMP_LIBRARIES}
	${BULLET_LIBRARIES}
	${CONNEXION_LIBRARIES}
)

if(BUILDGRAPHICS)
	set_target_properties(Planner PROPERTIES
	COMPILE_DEFINITIONS "WITHGRAPHICS")

	find_package(PkgConfig REQUIRED)
	pkg_search_module(GLFW REQUIRED glfw3)
	INCLUDE(FindOpenGL)
	INCLUDE(FindGLEW)

	include_directories(
		${GLFW_INCLUDE_DIRS}
		${GLEW_DIRS}
		${OPENGL_INCLUDE_DIRS}
	)

	if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

		FIND_LIBRARY(COCOA_LIBRARY Cocoa)
		FIND_LIBRARY(COREVIDEO_LIBRARY CoreVideo)
		FIND_LIBRARY(IOKIT_LIBRARY IOKit)

		target_link_libraries(Planner
			${GLFW_STATIC_LIBRARIES}
			${GLEW_LIBRARIES}
			${OPENGL_LIBRARIES}
			${COCOA_LIBRARY}
			${COREVIDEO_LIBRARY}
			${IOKIT_LIBRARY}
		)
	else(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
		target_link_libraries(Planner
			${GLFW_STATIC_LIBRARIES}
			${GLEW_LIBRARIES}
			${OPENGL_LIBRARIES}
		)
	endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
endif(BUILDGRAPHICS)



######### VREP Plugin ##########

add_library(v_repExtskiesel SHARED dependencies/V-REP_PRO_EDU_V3_2_1_Mac/programming/common/v_repLib.cpp vrep/plugin.cpp)

set_target_properties(v_repExtskiesel PROPERTIES
	COMPILE_DEFINITIONS "VREPPLUGIN")

ADD_CUSTOM_COMMAND(
	TARGET v_repExtskiesel
	POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy ARGS ${CMAKE_BINARY_DIR}/libv_repExtskiesel.dylib
	${CMAKE_BINARY_DIR}/../dependencies/V-REP_PRO_EDU_V3_2_1_Mac/vrep.app/Contents/MacOS/
)

target_link_libraries(v_repExtskiesel
	${Boost_LIBRARIES}
	${FCL_LIBRARIES}
)