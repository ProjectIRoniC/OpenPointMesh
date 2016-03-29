# - Try to find MNG
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``MNG::MNG``, if
# MNG has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  MNG_FOUND 			- system has MNG
#  MNG_INCLUDE_DIRS		- the MNG include directories
#  MNG_LIBRARIES		- link these to use MNG
#  MNG_DEFINITIONS		- compiler flags for MNG
#  MNG_VERSION			- the version of MNG found (x.y.z)

IF( MNG_FOUND )
	MESSAGE( STATUS "MNG is already in the cache." )
	return()
ENDIF()

INCLUDE( ${CMAKE_CURRENT_LIST_DIR}/LibFindMacros.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/SelectLibraryConfigurations.cmake )

# Dependencies
LIBFIND_PACKAGE( MNG ZLIB )
LIBFIND_PACKAGE( MNG JPEG )
LIBFIND_PACKAGE( MNG LCMS )

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
LIBFIND_PKG_CHECK_MODULES( MNG_PKGCONF mng )
SET( MNG_DEFINITIONS ${MNG_PKGCONF_CFLAGS_OTHER} )

# Include dir
FIND_PATH( MNG_INCLUDE_DIR
  NAMES libmng.h
  HINTS ${MNG_PKGCONF_INCLUDE_DIRS}
)

# The release library
FIND_LIBRARY( MNG_LIBRARY_RELEASE
  NAMES mng libmng
  HINTS ${MNG_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( MNG_LIBRARY_DEBUG
  NAMES mngd libmngd
  HINTS ${MNG_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( MNG_LIBRARY_RELEASE MNG_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( MNG )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( MNG_FOUND )
UNSET( MNG_LIBRARIES )

# The version number
LIBFIND_VERSION_HEADER( MNG libmng.h MNG_VERSION_TEXT )

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( MNG_PROCESS_INCLUDES ${MNG_INCLUDE_DIR} ${LCMS_INCLUDE_DIRS} ${JPEG_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS} )
SET( MNG_PROCESS_LIBS ${MNG_LIBRARY} ${LCMS_LIBRARIES} ${JPEG_LIBRARIES} ${ZLIB_LIBRARIES} )
LIBFIND_PROCESS( MNG )

# Set IMPORTED Targets
IF( MNG_FOUND )
	IF( NOT TARGET MNG::MNG )
		ADD_LIBRARY( MNG::MNG ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( MNG::MNG PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${MNG_INCLUDE_DIRS}"
			VERSION "${MNG_VERSION}" )

		IF( MNG_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET MNG::MNG APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( MNG::MNG PROPERTIES
				IMPORTED_LOCATION_RELEASE "${MNG_LIBRARY_RELEASE}" )
		ENDIF()

		IF( MNG_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET MNG::MNG APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( MNG::MNG PROPERTIES
				IMPORTED_LOCATION_DEBUG "${MNG_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT MNG_LIBRARY_RELEASE AND NOT MNG_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET MNG::MNG APPEND PROPERTY
				IMPORTED_LOCATION "${MNG_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${MNG_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${MNG_LIBRARIES}" )
	IF( MNG_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${MNG_DEFINITIONS}" )
	ENDIF()
ENDIF()
