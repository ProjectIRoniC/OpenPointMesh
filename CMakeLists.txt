## CMake settings
#-----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED( VERSION 3.1.0 FATAL_ERROR ) # 3.1.0 NEEDED for CMAKE_CXX_STANDARD support

CMAKE_POLICY( VERSION 3.1.0 )
IF( POLICY CMP0048 )
	CMAKE_POLICY( SET CMP0048 OLD )
ENDIF()
IF( POLICY CMP0017 )
	CMAKE_POLICY( SET CMP0017 OLD )
ENDIF()
FOREACH( p
	CMP0054 # CMake 3.1
	)
	IF( POLICY ${p} )
		CMAKE_POLICY( SET ${p} NEW )
	ENDIF()
ENDFOREACH()

#-----------------------------------------------------------------------------
SET( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH} )
INCLUDE( ExternalProject )
INCLUDE( ExternalProjectDependency )

#-----------------------------------------------------------------------------
# IF( APPLE )
# 	# Note: By setting CMAKE_OSX_* variables before any enable_language() or project() calls,
# 	#       we ensure that the bitness, and C++ standard library will be properly detected.
# 	INCLUDE( SlicerBlockSetCMakeOSXVariables )
# 	MARK_AS_SUPERBUILD(
# 		VARS CMAKE_OSX_ARCHITECTURES:STRING CMAKE_OSX_SYSROOT:PATH CMAKE_OSX_DEPLOYMENT_TARGET:STRING
# 		ALL_PROJECTS
# 		)
# ENDIF()



SET( PRIMARY_PROJECT_NAME OpenPointMesh )
## NOTE THERE SHOULD BE NO PROJECT STATEMENT HERE!
## This file acts as a simple switch to initiate
## two completely independant CMake build environments.
#-----------------------------------------------------------------------------
# Superbuild Option - Enabled by default
#                   Phase I:  ${PRIMARY_PROJECT_NAME}_SUPERBUILD is set to ON, and the
#                             supporting packages defined in "SuperBuild.cmake"
#                             are built.  The last package in "SuperBuild.cmake"
#                             to be built is a recursive call to this
#                             file with ${PRIMARY_PROJECT_NAME}_SUPERBUILD explicitly
#                             set to "OFF" to initiate Phase II
#
#                   Phase II: Build the ${PRIMARY_PROJECT_NAME}, referencing the support
#                             packages built in Phase I.
#-----------------------------------------------------------------------------
OPTION( ${PRIMARY_PROJECT_NAME}_SUPERBUILD "Build ${PRIMARY_PROJECT_NAME} and the projects it depends on via SuperBuild.cmake." ON )
MARK_AS_ADVANCED( ${PRIMARY_PROJECT_NAME}_SUPERBUILD )

#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Common build features for both the superbuild and the main build
#-----------------------------------------------------------------------------
INCLUDE( ${CMAKE_CURRENT_SOURCE_DIR}/Common.cmake )

#-----------------------------------------------------------------------------
# Superbuild script
#-----------------------------------------------------------------------------
IF( ${PRIMARY_PROJECT_NAME}_SUPERBUILD )
	PROJECT( SuperBuild_${PRIMARY_PROJECT_NAME} )  # <- NOTE: Project name for pre-requisites is different form main project
	IF( NOT CMAKE_CXX_STANDARD )
		SET( CMAKE_CXX_STANDARD 98 ) # Supported values are ``98``, ``11`` and ``14``.
	ENDIF()
	#-----------------------------------------------------------------------------
	# Sanity checks
	#------------------------------------------------------------------------------
	IF( WIN32 )
		SET( ${PROJECT_NAME}_ROOT_DIR_MAX_LENGTH 40 )
#		IF( NOT ${PROJECT_NAME}_SUPERBUILD )
#			STRING( LENGTH ${Slicer_BINARY_INNER_SUBDIR} _inner_subdir_length )
#			MATH( EXPR ${PROJECT_NAME}_ROOT_DIR_MAX_LENGTH "${${PROJECT_NAME}_ROOT_DIR_MAX_LENGTH} + ${_inner_subdir_length}" )
#		ENDIF()
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
	IF( NOT CMAKE_CXX_STANDARD )
		SET( CMAKE_CXX_STANDARD 98 ) # Supported values are ``98``, ``11`` and ``14``.
	ENDIF()
	INCLUDE( "${CMAKE_CURRENT_SOURCE_DIR}/${PRIMARY_PROJECT_NAME}.cmake" )
	RETURN()
ENDIF()

MESSAGE( FATAL_ERROR "You should never reach this point !" )

