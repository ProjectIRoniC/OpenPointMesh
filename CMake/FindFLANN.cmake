# - Try to find FLANN
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``FLANN::FLANN``, if
# FLANN has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  FLANN_FOUND 			- system has FLANN
#  FLANN_INCLUDE_DIRS	- the FLANN include directories
#  FLANN_LIBRARIES		- link these to use FLANN
#  FLANN_DEFINITIONS	- compiler flags for FLANN
#  FLANN_VERSION		- the version of FLANN found (x.y.z)

IF( FLANN_FOUND )
	MESSAGE( STATUS "FLANN is already in the cache." )
	return()
ENDIF()

INCLUDE( ${CMAKE_CURRENT_LIST_DIR}/LibFindMacros.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/SelectLibraryConfigurations.cmake )

# Help CMake choose Static vs Shared Libraries
# Since CMake search order prefers Shared Libraries we only need
# to change the search order for Static Libraries
IF( BUILD_SHARED_LIBS )
	SET( LIB_TYPE SHARED )
	
	# FLANN Shared Library Names
	SET( FLANN_RELEASE_NAME flann_cpp )
	SET( FLANN_DEBUG_NAME flann_cpp-gd )
ELSE()
	SET( LIB_TYPE STATIC )
	IF( WIN32 )
		SET( CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES} )
	ELSE()
		SET( CMAKE_FIND_LIBRARY_SUFFIXES .a ${CMAKE_FIND_LIBRARY_SUFFIXES} )
	ENDIF()
	
	# FLANN Static Library Names
	SET( FLANN_RELEASE_NAME flann_cpp_s )
	SET( FLANN_DEBUG_NAME flann_cpp_s-gd )
ENDIF()

# Use pkg-config to get hints about paths if it exists
LIBFIND_PKG_CHECK_MODULES( FLANN_PKGCONF flann )
SET( FLANN_DEFINITIONS ${FLANN_PKGCONF_CFLAGS_OTHER} )

# Include dir
FIND_PATH( FLANN_INCLUDE_DIR
  NAMES flann/flann.h
  HINTS ${FLANN_PKGCONF_INCLUDE_DIRS}
)

# The release library
FIND_LIBRARY( FLANN_LIBRARY_RELEASE
  NAMES ${FLANN_RELEASE_NAME}
  HINTS ${FLANN_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( FLANN_LIBRARY_DEBUG
  NAMES ${FLANN_DEBUG_NAME}
  HINTS ${FLANN_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( FLANN_LIBRARY_RELEASE FLANN_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( FLANN )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( FLANN_FOUND )
UNSET( FLANN_LIBRARIES )

# The version number
LIBFIND_VERSION_HEADER( FLANN flann/config.h FLANN_VERSION_ )

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( FLANN_PROCESS_INCLUDES ${FLANN_INCLUDE_DIR} )
SET( FLANN_PROCESS_LIBS ${FLANN_LIBRARY} )
LIBFIND_PROCESS( FLANN )

# Set IMPORTED Targets
IF( FLANN_FOUND )
	IF( NOT TARGET FLANN::FLANN )
		ADD_LIBRARY( FLANN::FLANN ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( FLANN::FLANN PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${FLANN_INCLUDE_DIRS}"
			VERSION "${FLANN_VERSION}" )

		IF( FLANN_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET FLANN::FLANN APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( FLANN::FLANN PROPERTIES
				IMPORTED_LOCATION_RELEASE "${FLANN_LIBRARY_RELEASE}" )
		ENDIF()

		IF( FLANN_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET FLANN::FLANN APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( FLANN::FLANN PROPERTIES
				IMPORTED_LOCATION_DEBUG "${FLANN_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT FLANN_LIBRARY_RELEASE AND NOT FLANN_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET FLANN::FLANN APPEND PROPERTY
				IMPORTED_LOCATION "${FLANN_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${FLANN_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${FLANN_LIBRARIES}" )
	IF( FLANN_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${FLANN_DEFINITIONS}" )
	ENDIF()
ENDIF()
