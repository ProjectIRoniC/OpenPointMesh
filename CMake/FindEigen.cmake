# - Try to find Eigen
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``Eigen::Eigen``, if
# Eigen has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  Eigen_FOUND 			- system has Eigen
#  Eigen_INCLUDE_DIRS	- the Eigen include directories
#  Eigen_VERSION		- the version of Eigen found (x.y.z)

IF( Eigen_FOUND )
	MESSAGE( STATUS "Eigen is already in the cache." )
	return()
ENDIF()

INCLUDE( ${CMAKE_CURRENT_LIST_DIR}/LibFindMacros.cmake )

# Use pkg-config to get hints about paths if it exists
LIBFIND_PKG_CHECK_MODULES( Eigen_PKGCONF eigen )

# Include dir
FIND_PATH( Eigen_INCLUDE_DIR
  NAMES signature_of_eigen3_matrix_library
  HINTS ${Eigen_PKGCONF_INCLUDE_DIRS}
  PATH_SUFFIXES eigen3
)

# The version number
IF( EXISTS "${Eigen_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h" )
	SET( Eigen_Macros_H "${Eigen_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h" )
ENDIF()

IF( Eigen_INCLUDE_DIR AND Eigen_Macros_H )
	FILE(READ "${Eigen_Macros_H}" _eigen3_version_header)

	STRING( REGEX MATCH "define[ \t]+EIGEN_WORLD_VERSION[ \t]+([0-9]+)" _eigen3_world_version_match "${_eigen3_version_header}" )
	SET( Eigen_WORLD_VERSION "${CMAKE_MATCH_1}" )
	STRING( REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)" _eigen3_major_version_match "${_eigen3_version_header}" )
	SET( Eigen_MAJOR_VERSION "${CMAKE_MATCH_1}" )
	STRING( REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)" _eigen3_minor_version_match "${_eigen3_version_header}" )
	SET( Eigen_MINOR_VERSION "${CMAKE_MATCH_1}" )

	SET( Eigen_VERSION ${Eigen_WORLD_VERSION}.${Eigen_MAJOR_VERSION}.${Eigen_MINOR_VERSION} )
ENDIF()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
# Eigen is a headers only library
SET( Eigen_PROCESS_INCLUDES ${Eigen_INCLUDE_DIR} )
LIBFIND_PROCESS( Eigen )

# Set IMPORTED Targets
IF( Eigen_FOUND )
	IF( NOT TARGET Eigen::Eigen )		
		ADD_LIBRARY( Eigen::Eigen UNKNOWN IMPORTED )
		SET_TARGET_PROPERTIES( Eigen::Eigen PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${Eigen_INCLUDE_DIRS}" )
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${Eigen_INCLUDE_DIRS}" )
ENDIF()
