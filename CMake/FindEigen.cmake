# - Try to find EIGEN
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``EIGEN::EIGEN``, if
# EIGEN has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  EIGEN_FOUND 			- system has EIGEN
#  EIGEN_INCLUDE_DIRS	- the EIGEN include directories
#  EIGEN_VERSION		- the version of EIGEN found (x.y.z)

IF( EIGEN_FOUND )
	MESSAGE( STATUS "EIGEN is already in the cache." )
	return()
ENDIF()

INCLUDE( ${CMAKE_CURRENT_LIST_DIR}/LibFindMacros.cmake )

# Use pkg-config to get hints about paths if it exists
LIBFIND_PKG_CHECK_MODULES( EIGEN_PKGCONF eigen )

# Include dir
FIND_PATH( EIGEN_INCLUDE_DIR
  NAMES Eigen/Core
  HINTS ${EIGEN_PKGCONF_INCLUDE_DIRS}
  PATH_SUFFIXES eigen3
)

# The version number
IF( EXISTS "${EIGEN_INCLUDE_DIR}/EIGEN/src/Core/util/Macros.h" )
	SET( EIGEN_Macros_H "${EIGEN_INCLUDE_DIR}/EIGEN/src/Core/util/Macros.h" )
ENDIF()

IF( EIGEN_INCLUDE_DIR AND EIGEN_Macros_H )
	FILE(READ "${EIGEN_Macros_H}" _eigen3_version_header)

	STRING( REGEX MATCH "define[ \t]+EIGEN_WORLD_VERSION[ \t]+([0-9]+)" _eigen3_world_version_match "${_eigen3_version_header}" )
	SET( EIGEN_WORLD_VERSION "${CMAKE_MATCH_1}" )
	STRING( REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)" _eigen3_major_version_match "${_eigen3_version_header}" )
	SET( EIGEN_MAJOR_VERSION "${CMAKE_MATCH_1}" )
	STRING( REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)" _eigen3_minor_version_match "${_eigen3_version_header}" )
	SET( EIGEN_MINOR_VERSION "${CMAKE_MATCH_1}" )

	SET( EIGEN_VERSION ${EIGEN_WORLD_VERSION}.${EIGEN_MAJOR_VERSION}.${EIGEN_MINOR_VERSION} )
ENDIF()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
# EIGEN is a headers only library
SET( EIGEN_PROCESS_INCLUDES ${EIGEN_INCLUDE_DIR} )
LIBFIND_PROCESS( EIGEN )

# Set IMPORTED Targets
IF( EIGEN_FOUND )
	IF( NOT TARGET EIGEN::EIGEN )		
		ADD_LIBRARY( EIGEN::EIGEN UNKNOWN IMPORTED )
		SET_TARGET_PROPERTIES( EIGEN::EIGEN PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_INCLUDE_DIRS}" )
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${EIGEN_INCLUDE_DIRS}" )
ENDIF()
