cmake_minimum_required(VERSION 2.8)

SET( PLUGIN_NAME Surface_DeformationCage )

INCLUDE_DIRECTORIES(
	include
	${CMAKE_CURRENT_BINARY_DIR}
)

file(
	GLOB_RECURSE
	PLUGIN_FILES
	src/*.cpp
	include/*.h
	include/*.hpp
)

file(
	GLOB_RECURSE
	PLUGIN_UI_FILES
	forms/*.ui
)

SET(
	PLUGIN_QOBJECT_FILES
	include/surface_deformationCage.h
	include/dialog_deformationCage.h
)

include( ${SCHNApps_ROOT_DIR}/Plugins/plugins_cmake.txt )
