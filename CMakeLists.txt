##############################################################################
# Top level CMakeLists file for the SuperBuild.
#
# This file is responsible for setting up the SuperBuild process which runs
# in two phases. The first phase runs SuperBuild.cmake which sets up the
# building of the dependencies of the Primary Project OpenPointMesh.  When
# SuperBuild.cmake is complete the second phase begins where this file is
# recursively called, but processes the Primary Project cmake file
# OpenPointMesh.cmake instead of SuperBuild.cmake.
##############################################################################
# CMake Policy settings
# 3.1.0 NEEDED for CMAKE_CXX_STANDARD support
#-----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED( VERSION 3.1.0 FATAL_ERROR )
 
#-----------------------------------------------------------------------------
# Add the CMake Module Path
#-----------------------------------------------------------------------------
SET( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH} )


#-----------------------------------------------------------------------------
# Setup the SuperBuild
#-----------------------------------------------------------------------------
SET( PRIMARY_PROJECT_NAME OpenPointMesh )
## NOTE THERE SHOULD BE NO PROJECT STATEMENT HERE!
## This file acts as a simple switch to initiate
## two completely independant CMake build environments.
#-----------------------------------------------------------------------------
# Superbuild Option - Enabled by default
#		Phase I:  ${PRIMARY_PROJECT_NAME}_SUPERBUILD is set to ON, and the
#					supporting packages defined in "SuperBuild.cmake"
#					are built.  The last package in "SuperBuild.cmake"
#					to be built is a recursive call to this
#					file with ${PRIMARY_PROJECT_NAME}_SUPERBUILD explicitly
#					set to "OFF" to initiate Phase II
#
#		Phase II: Build the ${PRIMARY_PROJECT_NAME}, referencing the support
#					packages built in Phase I.
#-----------------------------------------------------------------------------
OPTION( ${PRIMARY_PROJECT_NAME}_SUPERBUILD "Build ${PRIMARY_PROJECT_NAME} and the projects it depends on via SuperBuild.cmake." ON )
MARK_AS_ADVANCED( ${PRIMARY_PROJECT_NAME}_SUPERBUILD )


#-----------------------------------------------------------------------------
# Common variables and functionality for the Primary and Dependency Projects
#-----------------------------------------------------------------------------
INCLUDE( ${CMAKE_CURRENT_SOURCE_DIR}/Common.cmake )


#-----------------------------------------------------------------------------
# Superbuild script
#-----------------------------------------------------------------------------
IF( ${PRIMARY_PROJECT_NAME}_SUPERBUILD )
	PROJECT( SuperBuild_${PRIMARY_PROJECT_NAME} )  # <- NOTE: Project name for pre-requisites is different from main project
	# Sanity checks
	IF( WIN32 )
		SET( ${PROJECT_NAME}_ROOT_DIR_MAX_LENGTH 40 )
		IF( NOT ${PROJECT_NAME}_SUPERBUILD )
			STRING( LENGTH ${CMAKE_CURRENT_BINARY_DIR} _inner_subdir_length )
			MATH( EXPR ${PROJECT_NAME}_ROOT_DIR_MAX_LENGTH "${${PROJECT_NAME}_ROOT_DIR_MAX_LENGTH} + ${_inner_subdir_length}" )
		ENDIF()
		INCLUDE( PreventDirWithTooManyChars )
	ENDIF()
	INCLUDE( PreventInSourceBuilds )
	INCLUDE( PreventInBuildInstalls )
	INCLUDE( PreventDirWithSpaces )
	MARK_AS_SUPERBUILD( ${PROJECT_NAME}_SKIP_DIR_WITH_SPACES_CHECK:BOOL )

	INCLUDE( "${CMAKE_CURRENT_SOURCE_DIR}/SuperBuild.cmake" )
	RETURN()
ELSE()
	PROJECT( ${PRIMARY_PROJECT_NAME} ) # <- NOTE: Here is the main project name setting
	INCLUDE( "${CMAKE_CURRENT_SOURCE_DIR}/${PRIMARY_PROJECT_NAME}.cmake" )
	RETURN()
ENDIF()

MESSAGE( FATAL_ERROR "You should never reach this point !" )

