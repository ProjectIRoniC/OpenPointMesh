# - Try to find JPEG
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``JPEG::JPEG``, if
# JPEG has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  JPEG_FOUND 			- system has JPEG
#  JPEG_INCLUDE_DIRS	- the JPEG include directories
#  JPEG_LIBRARIES		- link these to use JPEG
#  JPEG_DEFINITIONS		- compiler flags for JPEG
#  JPEG_VERSION			- the version of JPEG found (x.y)

IF( JPEG_FOUND )
	MESSAGE( STATUS "JPEG is already in the cache." )
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
LIBFIND_PKG_CHECK_MODULES( JPEG_PKGCONF jpeg )
SET( JPEG_DEFINITIONS ${JPEG_PKGCONF_CFLAGS_OTHER} )

# Include dir
FIND_PATH( JPEG_INCLUDE_DIR
  NAMES jpeglib.h
  HINTS ${JPEG_PKGCONF_INCLUDE_DIRS}
)

# The release library
FIND_LIBRARY( JPEG_LIBRARY_RELEASE
  NAMES jpeg libjpeg
  HINTS ${JPEG_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( JPEG_LIBRARY_DEBUG
  NAMES jpegd libjpegd
  HINTS ${JPEG_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( JPEG_LIBRARY_RELEASE JPEG_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( JPEG )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( JPEG_FOUND )
UNSET( JPEG_LIBRARIES )

# The version number
IF( EXISTS "${JPEG_INCLUDE_DIR}/jpeglib.h" )
	SET( JPEG_H "${JPEG_INCLUDE_DIR}/jpeglib.h" )
ENDIF()

IF( JPEG_INCLUDE_DIR AND JPEG_H )
	FILE( STRINGS "${JPEG_H}" jpeg_version_str
		REGEX "^#[\t ]*define[\t ]+JPEG_LIB_VERSION_(MAJOR|MINOR)[\t ]+[0-9]+$" )

	UNSET( JPEG_VERSION )
	FOREACH( VPART MAJOR MINOR )
		FOREACH( VLINE ${jpeg_version_str} )
			IF( VLINE MATCHES "^#[\t ]*define[\t ]+JPEG_LIB_VERSION_${VPART}[\t ]+([0-9]+)$" )
				SET( JPEG_VERSION_PART "${CMAKE_MATCH_1}" )
				IF( JPEG_VERSION )
					SET( JPEG_VERSION "${JPEG_VERSION}.${JPEG_VERSION_PART}" )
				ELSE()
					SET( JPEG_VERSION "${JPEG_VERSION_PART}" )
				ENDIF()
				UNSET( JPEG_VERSION_PART )
			ENDIF()
		ENDFOREACH()
	ENDFOREACH()
ENDIF()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( JPEG_PROCESS_INCLUDES ${JPEG_INCLUDE_DIR} )
SET( JPEG_PROCESS_LIBS ${JPEG_LIBRARY} )
LIBFIND_PROCESS( JPEG )

# Set IMPORTED Targets
IF( JPEG_FOUND )
	IF( NOT TARGET JPEG::JPEG )
		ADD_LIBRARY( JPEG::JPEG ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( JPEG::JPEG PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${JPEG_INCLUDE_DIRS}"
			VERSION "${JPEG_VERSION}" )

		IF( JPEG_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET JPEG::JPEG APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( JPEG::JPEG PROPERTIES
				IMPORTED_LOCATION_RELEASE "${JPEG_LIBRARY_RELEASE}" )
		ENDIF()

		IF( JPEG_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET JPEG::JPEG APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( JPEG::JPEG PROPERTIES
				IMPORTED_LOCATION_DEBUG "${JPEG_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT JPEG_LIBRARY_RELEASE AND NOT JPEG_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET JPEG::JPEG APPEND PROPERTY
				IMPORTED_LOCATION "${JPEG_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${JPEG_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${JPEG_LIBRARIES}" )
	IF( JPEG_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${JPEG_DEFINITIONS}" )
	ENDIF()
ENDIF()
