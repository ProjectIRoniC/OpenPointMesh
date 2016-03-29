# - Try to find Freetype
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``Freetype::Freetype``, if
# Freetype has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  Freetype_FOUND 			- system has Freetype
#  Freetype_INCLUDE_DIRS	- the Freetype include directories
#  Freetype_LIBRARIES		- link these to use Freetype
#  Freetype_DEFINITIONS		- compiler flags for Freetype
#  Freetype_VERSION			- the version of Freetype found (x.y.z)

IF( Freetype_FOUND )
	MESSAGE( STATUS "Freetype is already in the cache." )
	return()
ENDIF()

INCLUDE( ${CMAKE_CURRENT_LIST_DIR}/LibFindMacros.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/SelectLibraryConfigurations.cmake )

# Dependencies
LIBFIND_PACKAGE( Freetype ZLIB )
LIBFIND_PACKAGE( Freetype BZip2 )
LIBFIND_PACKAGE( Freetype PNG )

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
LIBFIND_PKG_CHECK_MODULES( Freetype_PKGCONF freetype )
SET( Freetype_DEFINITIONS ${Freetype_PKGCONF_CFLAGS_OTHER} )

# Include dirs, Freetype does some include header tricks so there are two of them
FIND_PATH( Freetype_INCLUDE_DIR_ft2build
  NAMES ft2build.h
  HINTS ${Freetype_PKGCONF_INCLUDE_DIRS}
  PATH_SUFFIXES freetype2
)

FIND_PATH( Freetype_INCLUDE_DIR_freetype2
  NAMES freetype/config/ftheader.h
  HINTS ${Freetype_PKGCONF_INCLUDE_DIRS}
  PATH_SUFFIXES freetype2
)

# The release library
FIND_LIBRARY( Freetype_LIBRARY_RELEASE
  NAMES freetype libfreetype
  HINTS ${Freetype_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( Freetype_LIBRARY_DEBUG
  NAMES freetyped libfreetyped
  HINTS ${Freetype_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( Freetype_LIBRARY_RELEASE Freetype_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( Freetype )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( Freetype_FOUND )
UNSET( Freetype_LIBRARIES )

# The version number
IF( EXISTS "${Freetype_INCLUDE_DIR_ft2build}/freetype/freetype.h" )
	SET( Freetype_H "${Freetype_INCLUDE_DIR_ft2build}/freetype/freetype.h" )
ENDIF()

IF( Freetype_INCLUDE_DIR_ft2build AND Freetype_H )
	FILE( STRINGS "${Freetype_H}" freetype_version_str
		REGEX "^#[\t ]*define[\t ]+FREETYPE_(MAJOR|MINOR|PATCH)[\t ]+[0-9]+$" )
	
	UNSET( Freetype_VERSION )
	FOREACH( VPART MAJOR MINOR PATCH )
		FOREACH( VLINE ${freetype_version_str} )
			IF( VLINE MATCHES "^#[\t ]*define[\t ]+FREETYPE_${VPART}[\t ]+([0-9]+)$" )
				SET( Freetype_VERSION_PART "${CMAKE_MATCH_1}" )
				IF( Freetype_VERSION )
					SET( Freetype_VERSION "${Freetype_VERSION}.${Freetype_VERSION_PART}" )
				ELSE()
					SET( Freetype_VERSION "${Freetype_VERSION_PART}" )
				ENDIF()
				UNSET( Freetype_VERSION_PART )
			ENDIF()
		ENDFOREACH()
	ENDFOREACH()
ENDIF()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( Freetype_PROCESS_INCLUDES Freetype_INCLUDE_DIR_ft2build Freetype_INCLUDE_DIR_freetype2 ${PNG_INCLUDE_DIRS} ${BZip2_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS} )
SET( Freetype_PROCESS_LIBS ${Freetype_LIBRARY} ${PNG_LIBRARIES} ${BZip2_LIBRARIES} ${ZLIB_LIBRARIES} )
LIBFIND_PROCESS( Freetype )

# Set IMPORTED Targets
IF( Freetype_FOUND )
	IF( NOT TARGET Freetype::Freetype )
		ADD_LIBRARY( Freetype::Freetype ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( Freetype::Freetype PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${Freetype_INCLUDE_DIRS}"
			VERSION "${Freetype_VERSION}" )

		IF( Freetype_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET Freetype::Freetype APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( Freetype::Freetype PROPERTIES
				IMPORTED_LOCATION_RELEASE "${Freetype_LIBRARY_RELEASE}" )
		ENDIF()

		IF( Freetype_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET Freetype::Freetype APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( Freetype::Freetype PROPERTIES
				IMPORTED_LOCATION_DEBUG "${Freetype_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT Freetype_LIBRARY_RELEASE AND NOT Freetype_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET Freetype::Freetype APPEND PROPERTY
				IMPORTED_LOCATION "${Freetype_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${Freetype_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${Freetype_LIBRARIES}" )
	IF( Freetype_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${Freetype_DEFINITIONS}" )
	ENDIF()
ENDIF()
