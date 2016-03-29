# - Try to find TIFF
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``TIFF::TIFF``, if
# TIFF has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  TIFF_FOUND 			- system has TIFF
#  TIFF_INCLUDE_DIRS	- the TIFF include directories
#  TIFF_LIBRARIES		- link these to use TIFF
#  TIFF_DEFINITIONS		- compiler flags for TIFF
#  TIFF_VERSION			- the version of TIFF found (x.y.z)

IF( TIFF_FOUND )
	MESSAGE( STATUS "TIFF is already in the cache." )
	return()
ENDIF()

INCLUDE( ${CMAKE_CURRENT_LIST_DIR}/LibFindMacros.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/SelectLibraryConfigurations.cmake )

# Dependencies
LIBFIND_PACKAGE( TIFF ZLIB )

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
LIBFIND_PKG_CHECK_MODULES( TIFF_PKGCONF tiff )
SET( TIFF_DEFINITIONS ${TIFF_PKGCONF_CFLAGS_OTHER} )

# Include dir
FIND_PATH( TIFF_INCLUDE_DIR
  NAMES tiff.h
  HINTS ${TIFF_PKGCONF_INCLUDE_DIRS}
)

# The release library
FIND_LIBRARY( TIFF_LIBRARY_RELEASE
  NAMES tiff libtiff tiff3 libtiff3
  HINTS ${TIFF_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( TIFF_LIBRARY_DEBUG
  NAMES tiffd libtiffd tiff3d libtiff3d
  HINTS ${TIFF_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( TIFF_LIBRARY_RELEASE TIFF_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( TIFF )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( TIFF_FOUND )
UNSET( TIFF_LIBRARIES )

# The version number
IF( EXISTS "${TIFF_INCLUDE_DIR}/tiffvers.h" )
	SET( TIFF_H "${TIFF_INCLUDE_DIR}/tiffvers.h" )
ENDIF()

IF( TIFF_INCLUDE_DIR AND TIFF_H )
    FILE( STRINGS "${TIFF_H}" tiff_version_str
         REGEX "^#define[\t ]+TIFFLIB_VERSION_STR[\t ]+\"LIBTIFF, Version .*" )

    STRING( REGEX REPLACE "^#define[\t ]+TIFFLIB_VERSION_STR[\t ]+\"LIBTIFF, Version +([^ \\n]*).*"
           "\\1" TIFF_VERSION "${tiff_version_str}" )
    UNSET( tiff_version_str )
ENDIF()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( TIFF_PROCESS_INCLUDES ${TIFF_INCLUDE_DIR} ${ZLIB_INCLUDE_DIRS} )
SET( TIFF_PROCESS_LIBS ${TIFF_LIBRARY} ${ZLIB_LIBRARIES} )
LIBFIND_PROCESS( TIFF )

# Set IMPORTED Targets
IF( TIFF_FOUND )
	IF( NOT TARGET TIFF::TIFF )
		ADD_LIBRARY( TIFF::TIFF ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( TIFF::TIFF PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${TIFF_INCLUDE_DIRS}"
			VERSION "${TIFF_VERSION}" )

		IF( TIFF_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET TIFF::TIFF APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( TIFF::TIFF PROPERTIES
				IMPORTED_LOCATION_RELEASE "${TIFF_LIBRARY_RELEASE}" )
		ENDIF()

		IF( TIFF_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET TIFF::TIFF APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( TIFF::TIFF PROPERTIES
				IMPORTED_LOCATION_DEBUG "${TIFF_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT TIFF_LIBRARY_RELEASE AND NOT TIFF_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET TIFF::TIFF APPEND PROPERTY
				IMPORTED_LOCATION "${TIFF_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${TIFF_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${TIFF_LIBRARIES}" )
	IF( TIFF_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${TIFF_DEFINITIONS}" )
	ENDIF()
ENDIF()
