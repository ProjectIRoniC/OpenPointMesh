# - Try to find ZLIB
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``ZLIB::ZLIB``, if
# ZLIB has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  ZLIB_FOUND 			- system has ZLIB
#  ZLIB_INCLUDE_DIRS	- the ZLIB include directories
#  ZLIB_LIBRARIES		- link these to use ZLIB
#  ZLIB_DEFINITIONS		- compiler flags for ZLIB
#  ZLIB_VERSION			- the version of ZLIB found (x.y.z)

IF( ZLIB_FOUND )
	MESSAGE( STATUS "ZLIB is already in the cache." )
	return()
ENDIF()

INCLUDE( ${CMAKE_CURRENT_LIST_DIR}/LibFindMacros.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/SelectLibraryConfigurations.cmake )

# Help CMake choose Static vs Shared Libraries
# Since CMake search order prefers Shared Libraries we only need
# to change the search order for Static Libraries
IF( NOT BUILD_SHARED_LIBS )
	IF( WIN32 )
		SET( CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES} )
	ELSE()
		SET( CMAKE_FIND_LIBRARY_SUFFIXES .a ${CMAKE_FIND_LIBRARY_SUFFIXES} )
	ENDIF()
ENDIF()

# Use pkg-config to get hints about paths if it exists
LIBFIND_PKG_CHECK_MODULES( ZLIB_PKGCONF zlib )
SET( ZLIB_DEFINITIONS ${ZLIB_PKGCONF_CFLAGS_OTHER} )

# Include dir
FIND_PATH( ZLIB_INCLUDE_DIR
  NAMES zlib.h
  HINTS ${ZLIB_PKGCONF_INCLUDE_DIRS}
)

# The release library
FIND_LIBRARY( ZLIB_LIBRARY_RELEASE
  NAMES z zlib zdll zlib1
  HINTS ${ZLIB_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( ZLIB_LIBRARY_DEBUG
  NAMES zlibd zlibd1
  HINTS ${ZLIB_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( ZLIB_LIBRARY_RELEASE ZLIB_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( ZLIB )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( ZLIB_FOUND )
UNSET( ZLIB_LIBRARIES )

# The version number
LIBFIND_VERSION_HEADER( ZLIB zlib.h ZLIB_VERSION )

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( ZLIB_PROCESS_INCLUDES ${ZLIB_INCLUDE_DIR} )
SET( ZLIB_PROCESS_LIBS ${ZLIB_LIBRARY} )
LIBFIND_PROCESS( ZLIB )

# Set IMPORTED Targets
IF( ZLIB_FOUND )
	IF( NOT TARGET ZLIB::ZLIB )
		GET_FILENAME_COMPONENT( LIB_EXT ${ZLIB_LIBRARY} EXT )
		IF( "${LIB_EXT}" MATCHES "a" OR "${LIB_EXT}" MATCHES "lib" )
			SET( LIB_TYPE STATIC )
		ELSE()
			SET( LIB_TYPE SHARED )
		ENDIF()
		
		ADD_LIBRARY( ZLIB::ZLIB ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( ZLIB::ZLIB PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${ZLIB_INCLUDE_DIRS}"
			VERSION "${ZLIB_VERSION}" )

		IF( ZLIB_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET ZLIB::ZLIB APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( ZLIB::ZLIB PROPERTIES
				IMPORTED_LOCATION_RELEASE "${ZLIB_LIBRARY_RELEASE}" )
		ENDIF()

		IF( ZLIB_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET ZLIB::ZLIB APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( ZLIB::ZLIB PROPERTIES
				IMPORTED_LOCATION_DEBUG "${ZLIB_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT ZLIB_LIBRARY_RELEASE AND NOT ZLIB_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET ZLIB::ZLIB APPEND PROPERTY
				IMPORTED_LOCATION "${ZLIB_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${ZLIB_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${ZLIB_LIBRARIES}" )
	IF( ZLIB_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${ZLIB_DEFINITIONS}" )
	ENDIF()
ENDIF()
