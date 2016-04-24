# - Try to find Qhull
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``Qhull::Qhull``, if
# Qhull has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  Qhull_FOUND 			- system has Qhull
#  Qhull_INCLUDE_DIRS	- the Qhull include directories
#  Qhull_LIBRARIES		- link these to use Qhull
#  Qhull_DEFINITIONS	- compiler flags for Qhull

IF( Qhull_FOUND )
	MESSAGE( STATUS "Qhull is already in the cache." )
	return()
ENDIF()

INCLUDE( ${CMAKE_CURRENT_LIST_DIR}/LibFindMacros.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/SelectLibraryConfigurations.cmake )

# Help CMake choose Static vs Shared Libraries
# Since CMake search order prefers Shared Libraries we only need
# to change the search order for Static Libraries
IF( BUILD_SHARED_LIBS )
	SET( LIB_TYPE SHARED )
	
	# Qhull Shared Library Names
	SET( Qhull_RELEASE_NAME qhull_p )
	SET( Qhull_DEBUG_NAME qhull_pd )
	
	# If using shared, be sure to use libqhullcpp
	SET( Qhull_H Qhull.h )
ELSE()
	SET( LIB_TYPE STATIC )
	IF( WIN32 )
		SET( CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES} )
	ELSE()
		SET( CMAKE_FIND_LIBRARY_SUFFIXES .a ${CMAKE_FIND_LIBRARY_SUFFIXES} )
	ENDIF()
	
	# Qhull Static Library Names
	SET( Qhull_RELEASE_NAME qhullstatic_r )
	SET( Qhull_DEBUG_NAME qhullstatic_rd )
	
	# If using static, be sure to use libqhull_r
	SET( Qhull_H libqhull_r.h )
ENDIF()

# Use pkg-config to get hints about paths if it exists
LIBFIND_PKG_CHECK_MODULES( Qhull_PKGCONF qhull )
SET( Qhull_DEFINITIONS ${Qhull_PKGCONF_CFLAGS_OTHER} )

# Include dir
FIND_PATH( Qhull_INCLUDE_DIR
  NAMES ${Qhull_H}
  HINTS ${Qhull_PKGCONF_INCLUDE_DIRS}
  PATH_SUFFIXES libqhull_r libqhullcpp
)

# The release library
FIND_LIBRARY( Qhull_LIBRARY_RELEASE
  NAMES ${Qhull_RELEASE_NAME}
  HINTS ${Qhull_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( Qhull_LIBRARY_DEBUG
  NAMES ${Qhull_DEBUG_NAME}
  HINTS ${Qhull_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( Qhull_LIBRARY_RELEASE Qhull_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( Qhull )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( Qhull_FOUND )
UNSET( Qhull_LIBRARIES )

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( Qhull_PROCESS_INCLUDES ${Qhull_INCLUDE_DIR} )
SET( Qhull_PROCESS_LIBS ${Qhull_LIBRARY} )
LIBFIND_PROCESS( Qhull )

# Set IMPORTED Targets
IF( Qhull_FOUND )
	IF( NOT TARGET Qhull::Qhull )
		ADD_LIBRARY( Qhull::Qhull ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( Qhull::Qhull PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${Qhull_INCLUDE_DIRS}"
			VERSION "${Qhull_VERSION}" )

		IF( Qhull_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET Qhull::Qhull APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( Qhull::Qhull PROPERTIES
				IMPORTED_LOCATION_RELEASE "${Qhull_LIBRARY_RELEASE}" )
		ENDIF()

		IF( Qhull_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET Qhull::Qhull APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( Qhull::Qhull PROPERTIES
				IMPORTED_LOCATION_DEBUG "${Qhull_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT Qhull_LIBRARY_RELEASE AND NOT Qhull_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET Qhull::Qhull APPEND PROPERTY
				IMPORTED_LOCATION "${Qhull_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${Qhull_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${Qhull_LIBRARIES}" )
	IF( Qhull_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${Qhull_DEFINITIONS}" )
	ENDIF()
ENDIF()
