# - Try to find XnLib
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``XnLib::XnLib``, if
# XnLib has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  XnLib_FOUND 			- system has XnLib
#  XnLib_INCLUDE_DIRS	- the XnLib include directories
#  XnLib_LIBRARIES		- link these to use XnLib
#  XnLib_DEFINITIONS	- compiler flags for XnLib
#  XnLib_VERSION		- the version of XnLib found (x.y.z)

IF( XnLib_FOUND )
	MESSAGE( STATUS "XnLib is already in the cache." )
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
LIBFIND_PKG_CHECK_MODULES( XnLib_PKGCONF xnlib )
SET( XnLib_DEFINITIONS ${XnLib_PKGCONF_CFLAGS_OTHER} )

# Include dir
FIND_PATH( XnLib_INCLUDE_DIR
  NAMES XnLib.h
  HINTS ${XnLib_PKGCONF_INCLUDE_DIRS}
)

# The release library
FIND_LIBRARY( XnLib_LIBRARY_RELEASE
  NAMES XnLib
  HINTS ${XnLib_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( XnLib_LIBRARY_DEBUG
  NAMES XnLibd
  HINTS ${XnLib_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( XnLib_LIBRARY_RELEASE XnLib_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( XnLib )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( XnLib_FOUND )
UNSET( XnLib_LIBRARIES )

# The version number
IF( EXISTS "${XnLib_INCLUDE_DIR}/XnPsVersion.h" )
	SET( XnLib_H "${XnLib_INCLUDE_DIR}/XnPsVersion.h" )
ENDIF()

IF( XnLib_INCLUDE_DIR AND XnLib_H )
	FILE( STRINGS "${XnLib_H}" xnlib_version_str
		REGEX "^#[\t ]*define[\t ]+XN_PS_(MAJOR|MINOR|MAINTENANCE)_VERSION[\t ]+[0-9]+$" )
	
	UNSET( XnLib_VERSION )
	FOREACH( VPART MAJOR MINOR MAINTENANCE )
		FOREACH( VLINE ${xnlib_version_str} )
			IF( VLINE MATCHES "^#[\t ]*define[\t ]+XN_PS_${VPART}_VERSION[\t ]+([0-9]+)$" )
				SET( XnLib_VERSION_PART "${CMAKE_MATCH_1}" )
				IF( XnLib_VERSION )
					SET( XnLib_VERSION "${XnLib_VERSION}.${XnLib_VERSION_PART}" )
				ELSE()
					SET( XnLib_VERSION "${XnLib_VERSION_PART}" )
				ENDIF()
				UNSET( XnLib_VERSION_PART )
			ENDIF()
		ENDFOREACH()
	ENDFOREACH()
ENDIF()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( XnLib_PROCESS_INCLUDES ${XnLib_INCLUDE_DIR} )
SET( XnLib_PROCESS_LIBS ${XnLib_LIBRARY} )
LIBFIND_PROCESS( XnLib )

# Set IMPORTED Targets
IF( XnLib_FOUND )
	IF( NOT TARGET XnLib::XnLib )
		GET_FILENAME_COMPONENT( LIB_EXT ${XnLib_LIBRARY} EXT )
		IF( "${LIB_EXT}" MATCHES "a" OR "${LIB_EXT}" MATCHES "lib" )
			SET( LIB_TYPE STATIC )
		ELSE()
			SET( LIB_TYPE SHARED )
		ENDIF()
		
		ADD_LIBRARY( XnLib::XnLib ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( XnLib::XnLib PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${XnLib_INCLUDE_DIRS}"
			VERSION "${XnLib_VERSION}" )

		IF( XnLib_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET XnLib::XnLib APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( XnLib::XnLib PROPERTIES
				IMPORTED_LOCATION_RELEASE "${XnLib_LIBRARY_RELEASE}" )
		ENDIF()

		IF( XnLib_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET XnLib::XnLib APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( XnLib::XnLib PROPERTIES
				IMPORTED_LOCATION_DEBUG "${XnLib_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT XnLib_LIBRARY_RELEASE AND NOT XnLib_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET XnLib::XnLib APPEND PROPERTY
				IMPORTED_LOCATION "${XnLib_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${XnLib_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${XnLib_LIBRARIES}" )
	IF( XnLib_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${XnLib_DEFINITIONS}" )
	ENDIF()
ENDIF()
