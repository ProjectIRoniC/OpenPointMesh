# - Try to find FONTCONFIG
#
# NOTE: The FontConfig variables in this file are all UPPERCASE because VTK requires it
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``FONTCONFIG::FONTCONFIG``, if
# FONTCONFIG has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  FONTCONFIG_FOUND 			- system has FONTCONFIG
#  FONTCONFIG_INCLUDE_DIRS		- the FONTCONFIG include directories
#  FONTCONFIG_LIBRARIES			- link these to use FONTCONFIG
#  FONTCONFIG_DEFINITIONS		- compiler flags for FONTCONFIG
#  FONTCONFIG_VERSION			- the version of FONTCONFIG found (Version ID)

IF( FONTCONFIG_FOUND )
	MESSAGE( STATUS "FONTCONFIG is already in the cache." )
	return()
ENDIF()

INCLUDE( ${CMAKE_CURRENT_LIST_DIR}/LibFindMacros.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/SelectLibraryConfigurations.cmake )

# Dependencies
LIBFIND_PACKAGE( FONTCONFIG Freetype )
LIBFIND_PACKAGE( FONTCONFIG EXPAT )

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
LIBFIND_PKG_CHECK_MODULES( FONTCONFIG_PKGCONF fontconfig )
SET( FONTCONFIG_DEFINITIONS ${FONTCONFIG_PKGCONF_CFLAGS_OTHER} )

# Include dir
FIND_PATH( FONTCONFIG_INCLUDE_DIR
  NAMES fontconfig/fontconfig.h
  HINTS ${FONTCONFIG_PKGCONF_INCLUDE_DIRS}
)

# The release library
FIND_LIBRARY( FONTCONFIG_LIBRARY_RELEASE
  NAMES fontconfig libfontconfig
  HINTS ${FONTCONFIG_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( FONTCONFIG_LIBRARY_DEBUG
  NAMES fontconfigd libfontconfigd
  HINTS ${FONTCONFIG_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( FONTCONFIG_LIBRARY_RELEASE FONTCONFIG_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( FONTCONFIG )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( FONTCONFIG_FOUND )
UNSET( FONTCONFIG_LIBRARIES )

# The version number
IF( EXISTS "${FONTCONFIG_INCLUDE_DIR}/fontconfig/fontconfig.h" )
	SET( FONTCONFIG_H "${FONTCONFIG_INCLUDE_DIR}/fontconfig/fontconfig.h" )
ENDIF()

IF( FONTCONFIG_INCLUDE_DIR AND FONTCONFIG_H )
	FILE( STRINGS "${FONTCONFIG_H}" fontconfig_version_str
		REGEX "^#[\t ]*define[\t ]+FC_(MAJOR|MINOR|REVISION)[\t ]+[0-9]+$" )

	UNSET( FONTCONFIG_VERSION )
	FOREACH( VPART MAJOR MINOR REVISION )
		FOREACH( VLINE ${fontconfig_version_str} )
			IF( VLINE MATCHES "^#[\t ]*define[\t ]+FC_${VPART}[\t ]+([0-9]+)$" )
				SET( FONTCONFIG_VERSION_PART "${CMAKE_MATCH_1}" )
				IF( FONTCONFIG_VERSION )
					SET( FONTCONFIG_VERSION "${FONTCONFIG_VERSION}.${FONTCONFIG_VERSION_PART}" )
				ELSE()
					SET( FONTCONFIG_VERSION "${FONTCONFIG_VERSION_PART}" )
				ENDIF()
				UNSET( FONTCONFIG_VERSION_PART )
			ENDIF()
		ENDFOREACH()
	ENDFOREACH()
ENDIF()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( FONTCONFIG_PROCESS_INCLUDES ${FONTCONFIG_INCLUDE_DIR} ${Freetype_INCLUDE_DIRS} ${EXPAT_INCLUDE_DIRS} )
SET( FONTCONFIG_PROCESS_LIBS ${FONTCONFIG_LIBRARY} ${Freetype_LIBRARIES} ${EXPAT_LIBRARIES} )
LIBFIND_PROCESS( FONTCONFIG )

# Set IMPORTED Targets
IF( FONTCONFIG_FOUND )
	IF( NOT TARGET FONTCONFIG::FONTCONFIG )
		GET_FILENAME_COMPONENT( LIB_EXT ${FONTCONFIG_LIBRARY} EXT )
		IF( "${LIB_EXT}" MATCHES "a" OR "${LIB_EXT}" MATCHES "lib" )
			SET( LIB_TYPE STATIC )
		ELSE()
			SET( LIB_TYPE SHARED )
		ENDIF()
		
		ADD_LIBRARY( FONTCONFIG::FONTCONFIG ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( FONTCONFIG::FONTCONFIG PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${FONTCONFIG_INCLUDE_DIRS}"
			VERSION "${FONTCONFIG_VERSION}" )

		IF( FONTCONFIG_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET FONTCONFIG::FONTCONFIG APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( FONTCONFIG::FONTCONFIG PROPERTIES
				IMPORTED_LOCATION_RELEASE "${FONTCONFIG_LIBRARY_RELEASE}" )
		ENDIF()

		IF( FONTCONFIG_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET FONTCONFIG::FONTCONFIG APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( FONTCONFIG::FONTCONFIG PROPERTIES
				IMPORTED_LOCATION_DEBUG "${FONTCONFIG_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT FONTCONFIG_LIBRARY_RELEASE AND NOT FONTCONFIG_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET FONTCONFIG::FONTCONFIG APPEND PROPERTY
				IMPORTED_LOCATION "${FONTCONFIG_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${FONTCONFIG_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${FONTCONFIG_LIBRARIES}" )
	IF( FONTCONFIG_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${FONTCONFIG_DEFINITIONS}" )
	ENDIF()
ENDIF()
