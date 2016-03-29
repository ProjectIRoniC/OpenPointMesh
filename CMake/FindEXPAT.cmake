# - Try to find EXPAT
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``EXPAT::EXPAT``, if
# EXPAT has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  EXPAT_FOUND 			- system has EXPAT
#  EXPAT_INCLUDE_DIRS	- the EXPAT include directories
#  EXPAT_LIBRARIES		- link these to use EXPAT
#  EXPAT_DEFINITIONS	- compiler flags for EXPAT
#  EXPAT_VERSION		- the version of EXPAT found (x.y.z)

IF( EXPAT_FOUND )
	MESSAGE( STATUS "EXPAT is already in the cache." )
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
LIBFIND_PKG_CHECK_MODULES( EXPAT_PKGCONF expat )
SET( EXPAT_DEFINITIONS ${EXPAT_PKGCONF_CFLAGS_OTHER} )

# Include dir
FIND_PATH( EXPAT_INCLUDE_DIR
  NAMES expat.h
  HINTS ${EXPAT_PKGCONF_INCLUDE_DIRS}
)

# The release library
FIND_LIBRARY( EXPAT_LIBRARY_RELEASE
  NAMES expat libexpat
  HINTS ${EXPAT_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( EXPAT_LIBRARY_DEBUG
  NAMES expatd libexpatd
  HINTS ${EXPAT_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( EXPAT_LIBRARY_RELEASE EXPAT_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( EXPAT )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( EXPAT_FOUND )
UNSET( EXPAT_LIBRARIES )

# The version number
IF( EXISTS "${EXPAT_INCLUDE_DIR}/expat.h" )
	SET( EXPAT_H "${EXPAT_INCLUDE_DIR}/expat.h" )
ENDIF()

IF( EXPAT_INCLUDE_DIR AND EXPAT_H )
    FILE( STRINGS "${EXPAT_INCLUDE_DIR}/expat.h" expat_version_str
         REGEX "^#[\t ]*define[\t ]+XML_(MAJOR|MINOR|MICRO)_VERSION[\t ]+[0-9]+$" )

    UNSET( EXPAT_VERSION_STRING )
    FOREACH( VPART MAJOR MINOR MICRO )
        FOREACH( VLINE ${expat_version_str} )
            IF( VLINE MATCHES "^#[\t ]*define[\t ]+XML_${VPART}_VERSION[\t ]+([0-9]+)$" )
                SET( EXPAT_VERSION_PART "${CMAKE_MATCH_1}" )
                IF( EXPAT_VERSION_STRING )
                    SET( EXPAT_VERSION_STRING "${EXPAT_VERSION_STRING}.${EXPAT_VERSION_PART}" )
                ELSE()
                    SET( EXPAT_VERSION_STRING "${EXPAT_VERSION_PART}" )
                ENDIF()
            ENDIF()
        ENDFOREACH()
    ENDFOREACH()
ENDIF()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( EXPAT_PROCESS_INCLUDES ${EXPAT_INCLUDE_DIR} )
SET( EXPAT_PROCESS_LIBS ${EXPAT_LIBRARY} )
LIBFIND_PROCESS( EXPAT )

# Set IMPORTED Targets
IF( EXPAT_FOUND )
	IF( NOT TARGET EXPAT::EXPAT )
		ADD_LIBRARY( EXPAT::EXPAT ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( EXPAT::EXPAT PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${EXPAT_INCLUDE_DIRS}"
			VERSION "${EXPAT_VERSION}" )

		IF( EXPAT_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET EXPAT::EXPAT APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( EXPAT::EXPAT PROPERTIES
				IMPORTED_LOCATION_RELEASE "${EXPAT_LIBRARY_RELEASE}" )
		ENDIF()

		IF( EXPAT_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET EXPAT::EXPAT APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( EXPAT::EXPAT PROPERTIES
				IMPORTED_LOCATION_DEBUG "${EXPAT_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT EXPAT_LIBRARY_RELEASE AND NOT EXPAT_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET EXPAT::EXPAT APPEND PROPERTY
				IMPORTED_LOCATION "${EXPAT_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${EXPAT_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${EXPAT_LIBRARIES}" )
	IF( EXPAT_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${EXPAT_DEFINITIONS}" )
	ENDIF()
ENDIF()
