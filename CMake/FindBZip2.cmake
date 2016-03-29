# - Try to find BZip2
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``BZip2::BZip2``, if
# BZip2 has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  BZip2_FOUND 				- system has BZip2
#  BZip2_INCLUDE_DIRS		- the BZip2 include directories
#  BZip2_LIBRARIES			- link these to use BZip2
#  BZip2_DEFINITIONS		- compiler flags for BZip2

IF( BZip2_FOUND )
	MESSAGE( STATUS "BZip2 is already in the cache." )
	return()
ENDIF()

INCLUDE( ${CMAKE_CURRENT_LIST_DIR}/LibFindMacros.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/SelectLibraryConfigurations.cmake )

# Help CMake choose Static vs Shared Libraries
# Since CMake search order prefers Shared Libraries we only need
# to change the search order for Static Libraries
IF( BUILD_SHARED_LIBS )
	SET( LIB_TYPE SHARED )
ELSE()
	SET( LIB_TYPE STATIC )
	IF( WIN32 )
		SET( CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES} )
	ELSE()
		SET( CMAKE_FIND_LIBRARY_SUFFIXES .a ${CMAKE_FIND_LIBRARY_SUFFIXES} )
	ENDIF()
ENDIF()

# Use pkg-config to get hints about paths if it exists
LIBFIND_PKG_CHECK_MODULES( BZip2_PKGCONF bz2 )
SET( BZip2_DEFINITIONS ${BZip2_PKGCONF_CFLAGS_OTHER} )

# Include dir
FIND_PATH( BZip2_INCLUDE_DIR
  NAMES bzlib.h
  HINTS ${BZip2_PKGCONF_INCLUDE_DIRS}
)

# The release library
FIND_LIBRARY( BZip2_LIBRARY_RELEASE
  NAMES bz2 libbz2
  HINTS ${BZip2_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( BZip2_LIBRARY_DEBUG
  NAMES bz2d libbz2d
  HINTS ${BZip2_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( BZip2_LIBRARY_RELEASE BZip2_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( BZip2 )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( BZip2_FOUND )
UNSET( BZip2_LIBRARIES )

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( BZip2_PROCESS_INCLUDES ${BZip2_INCLUDE_DIR} )
SET( BZip2_PROCESS_LIBS ${BZip2_LIBRARY} )
LIBFIND_PROCESS( BZip2 )

# Set IMPORTED Targets
IF( BZip2_FOUND )
	IF( NOT TARGET BZip2::BZip2 )
		ADD_LIBRARY( BZip2::BZip2 ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( BZip2::BZip2 PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${BZip2_INCLUDE_DIRS}" )

		IF( BZip2_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET BZip2::BZip2 APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( BZip2::BZip2 PROPERTIES
				IMPORTED_LOCATION_RELEASE "${BZip2_LIBRARY_RELEASE}" )
		ENDIF()

		IF( BZip2_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET BZip2::BZip2 APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( BZip2::BZip2 PROPERTIES
				IMPORTED_LOCATION_DEBUG "${BZip2_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT BZip2_LIBRARY_RELEASE AND NOT BZip2_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET BZip2::BZip2 APPEND PROPERTY
				IMPORTED_LOCATION "${BZip2_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${BZip2_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${BZip2_LIBRARIES}" )
	IF( BZip2_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${BZip2_DEFINITIONS}" )
	ENDIF()
ENDIF()
