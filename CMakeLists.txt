cmake_minimum_required (VERSION 2.6)
project (MotionPlanning)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(BUILDGRAPHICS 1)

add_executable(Planner main.cpp)

find_package(PkgConfig REQUIRED)
pkg_search_module(FCL REQUIRED fcl)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.57.0 COMPONENTS thread system REQUIRED)

find_package(ASSIMP REQUIRED)

include_directories(
	${Boost_INCLUDE_DIRS}
	${Boost_LIBRARY_DIRS}
	${FCL_LIBRARY_DIRS}
	${ASSIMP_LIBRARY_DIRS}
)

target_link_libraries(Planner
	${Boost_LIBRARIES}
	${FCL_LIBRARIES}
	${ASSIMP_LIBRARIES}
)

if(BUILDGRAPHICS)
	add_definitions(-DWITHGRAPHICS)

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