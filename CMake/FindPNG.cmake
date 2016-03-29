# - Try to find PNG
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``PNG::PNG``, if
# PNG has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  PNG_FOUND 			- system has PNG
#  PNG_INCLUDE_DIRS		- the PNG include directories
#  PNG_LIBRARIES		- link these to use PNG
#  PNG_DEFINITIONS		- compiler flags for PNG
#  PNG_VERSION			- the version of PNG found (x.y.z)

IF( PNG_FOUND )
	MESSAGE( STATUS "PNG is already in the cache." )
	return()
ENDIF()

INCLUDE( ${CMAKE_CURRENT_LIST_DIR}/LibFindMacros.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/SelectLibraryConfigurations.cmake )

# Dependencies
LIBFIND_PACKAGE( PNG ZLIB )

# Help CMake choose Static vs Shared Libraries
# Since CMake search order prefers Shared Libraries we only need
# to change the search order for Static Libraries
IF( BUILD_SHARED_LIBS )
	SET( LIB_TYPE SHARED )
	SET( PNG_DEFINITIONS -DPNG_USE_DLL )
ELSE()
	SET( LIB_TYPE STATIC )
	SET( PNG_DEFINITIONS -DPNG_STATIC )
	IF( WIN32 )
		SET( CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES} )
	ELSE()
		SET( CMAKE_FIND_LIBRARY_SUFFIXES .a ${CMAKE_FIND_LIBRARY_SUFFIXES} )
	ENDIF()
ENDIF()

# Use pkg-config to get hints about paths if it exists
LIBFIND_PKG_CHECK_MODULES( PNG_PKGCONF png )
LIST( APPEND PNG_DEFINITIONS ${PNG_PKGCONF_CFLAGS_OTHER} )

# Include dir
FIND_PATH( PNG_INCLUDE_DIR
  NAMES png.h
  HINTS ${PNG_PKGCONF_INCLUDE_DIRS}
)

# The release library
FIND_LIBRARY( PNG_LIBRARY_RELEASE
  NAMES png libpng
  HINTS ${PNG_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( PNG_LIBRARY_DEBUG
  NAMES pngd libpngd
  HINTS ${PNG_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( PNG_LIBRARY_RELEASE PNG_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( PNG )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( PNG_FOUND )
UNSET( PNG_LIBRARIES )

# The version number
LIBFIND_VERSION_HEADER( PNG png.h PNG_LIBPNG_VER_STRING )

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( PNG_PROCESS_INCLUDES ${PNG_INCLUDE_DIR} ${ZLIB_INCLUDE_DIRS} )
SET( PNG_PROCESS_LIBS ${PNG_LIBRARY} ${ZLIB_LIBRARIES} )
LIBFIND_PROCESS( PNG )

# Set IMPORTED Targets
IF( PNG_FOUND )
	IF( NOT TARGET PNG::PNG )
		ADD_LIBRARY( PNG::PNG ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( PNG::PNG PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${PNG_INCLUDE_DIRS}"
			VERSION "${PNG_VERSION}" )

		IF( PNG_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET PNG::PNG APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( PNG::PNG PROPERTIES
				IMPORTED_LOCATION_RELEASE "${PNG_LIBRARY_RELEASE}" )
		ENDIF()

		IF( PNG_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET PNG::PNG APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( PNG::PNG PROPERTIES
				IMPORTED_LOCATION_DEBUG "${PNG_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT PNG_LIBRARY_RELEASE AND NOT PNG_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET PNG::PNG APPEND PROPERTY
				IMPORTED_LOCATION "${PNG_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${PNG_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${PNG_LIBRARIES}" )
	IF( PNG_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${PNG_DEFINITIONS}" )
	ENDIF()
ENDIF()
