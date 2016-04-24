# - Try to find LCMS
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``LCMS::LCMS``, if
# LCMS has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  LCMS_FOUND 			- system has LCMS
#  LCMS_INCLUDE_DIRS	- the LCMS include directories
#  LCMS_LIBRARIES		- link these to use LCMS
#  LCMS_DEFINITIONS		- compiler flags for LCMS
#  LCMS_VERSION			- the version of LCMS found (Version ID)

IF( LCMS_FOUND )
	MESSAGE( STATUS "LCMS is already in the cache." )
	return()
ENDIF()

INCLUDE( ${CMAKE_CURRENT_LIST_DIR}/LibFindMacros.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/SelectLibraryConfigurations.cmake )

# Dependencies
LIBFIND_PACKAGE( LCMS ZLIB )
LIBFIND_PACKAGE( LCMS JPEG )
LIBFIND_PACKAGE( LCMS TIFF )

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
LIBFIND_PKG_CHECK_MODULES( LCMS_PKGCONF lcms )
SET( LCMS_DEFINITIONS ${LCMS_PKGCONF_CFLAGS_OTHER} )

# Include dir
FIND_PATH( LCMS_INCLUDE_DIR
  NAMES lcms.h
  HINTS ${LCMS_PKGCONF_INCLUDE_DIRS}
)

# The release library
FIND_LIBRARY( LCMS_LIBRARY_RELEASE
  NAMES lcms liblcms
  HINTS ${LCMS_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( LCMS_LIBRARY_DEBUG
  NAMES lcmsd liblcmsd
  HINTS ${LCMS_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( LCMS_LIBRARY_RELEASE LCMS_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( LCMS )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( LCMS_FOUND )
UNSET( LCMS_LIBRARIES )

# The version number
LIBFIND_VERSION_HEADER( LCMS lcms.h LCMS_VERSION )

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( LCMS_PROCESS_INCLUDES ${LCMS_INCLUDE_DIR} ${TIFF_INCLUDE_DIRS} ${JPEG_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS} )
SET( LCMS_PROCESS_LIBS ${LCMS_LIBRARY} ${TIFF_LIBRARIES} ${JPEG_LIBRARIES} ${ZLIB_LIBRARIES} )
LIBFIND_PROCESS( LCMS )

# Set IMPORTED Targets
IF( LCMS_FOUND )
	IF( NOT TARGET LCMS::LCMS )
		GET_FILENAME_COMPONENT( LIB_EXT ${LCMS_LIBRARY} EXT )
		IF( "${LIB_EXT}" MATCHES "a" OR "${LIB_EXT}" MATCHES "lib" )
			SET( LIB_TYPE STATIC )
		ELSE()
			SET( LIB_TYPE SHARED )
		ENDIF()
		
		ADD_LIBRARY( LCMS::LCMS ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( LCMS::LCMS PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${LCMS_INCLUDE_DIRS}"
			VERSION "${LCMS_VERSION}" )

		IF( LCMS_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET LCMS::LCMS APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( LCMS::LCMS PROPERTIES
				IMPORTED_LOCATION_RELEASE "${LCMS_LIBRARY_RELEASE}" )
		ENDIF()

		IF( LCMS_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET LCMS::LCMS APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( LCMS::LCMS PROPERTIES
				IMPORTED_LOCATION_DEBUG "${LCMS_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT LCMS_LIBRARY_RELEASE AND NOT LCMS_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET LCMS::LCMS APPEND PROPERTY
				IMPORTED_LOCATION "${LCMS_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${LCMS_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${LCMS_LIBRARIES}" )
	IF( LCMS_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${LCMS_DEFINITIONS}" )
	ENDIF()
ENDIF()
