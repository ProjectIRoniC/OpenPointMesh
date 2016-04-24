# - Try to find OpenNI2
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
# This module defines :prop_tgt:`IMPORTED` target ``OpenNI2::OpenNI2``, if
# OpenNI2 has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
# This module defines the following variables:
#
#  OpenNI2_FOUND 			- system has OpenNI2
#  OpenNI2_INCLUDE_DIRS		- the OpenNI2 include directories
#  OpenNI2_LIBRARIES		- link these to use OpenNI2
#  OpenNI2_DEFINITIONS		- compiler flags for OpenNI2
#  OpenNI2_VERSION			- the version of OpenNI2 found (x.y.z)

IF( OpenNI2_FOUND )
	MESSAGE( STATUS "OpenNI2 is already in the cache." )
	return()
ENDIF()

INCLUDE( ${CMAKE_CURRENT_LIST_DIR}/LibFindMacros.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/SelectLibraryConfigurations.cmake )

# Dependencies
LIBFIND_PACKAGE( OpenNI2 XnLib )
LIBFIND_PACKAGE( OpenNI2 libusb-1.0 )

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
LIBFIND_PKG_CHECK_MODULES( OpenNI2_PKGCONF openni2 )
SET( OpenNI2_DEFINITIONS ${OpenNI2_PKGCONF_CFLAGS_OTHER} )

# Include dir
FIND_PATH( OpenNI2_INCLUDE_DIR
  NAMES OpenNI.h
  HINTS ${OpenNI2_PKGCONF_INCLUDE_DIRS}
)

# The release library
FIND_LIBRARY( OpenNI2_LIBRARY_RELEASE
  NAMES OpenNI2
  HINTS ${OpenNI2_PKGCONF_LIBRARY_DIRS}
)

# The debug library
FIND_LIBRARY( OpenNI2_LIBRARY_DEBUG
  NAMES OpenNI2d
  HINTS ${OpenNI2_PKGCONF_LIBRARY_DIRS}
)

MARK_AS_ADVANCED( OpenNI2_LIBRARY_RELEASE OpenNI2_LIBRARY_DEBUG )

# This macro takes a library base name as an argument, and will choose good values
# for basename_LIBRARY, basename_LIBRARIES, basename_LIBRARY_DEBUG, and
# basename_LIBRARY_RELEASE depending on what has been found and set.
SELECT_LIBRARY_CONFIGURATIONS( OpenNI2 )

# Set by SELECT_LIBRARY_CONFIGURATIONS(), but we want the ones from LIBFIND_PROCESS() below.
UNSET( OpenNI2_FOUND )
UNSET( OpenNI2_LIBRARIES )

# The version number
IF( EXISTS "${OpenNI2_INCLUDE_DIR}/OniVersion.h" )
	SET( OpenNI2_H "${OpenNI2_INCLUDE_DIR}/OniVersion.h" )
ENDIF()

IF( OpenNI2_INCLUDE_DIR AND OpenNI2_H )
	FILE( STRINGS "${OpenNI2_H}" openni2_version_str
		REGEX "^#[\t ]*define[\t ]+ONI_VERSION_(MAJOR|MINOR|MAINTENANCE)[\t ]+[0-9]+$" )
	
	UNSET( OpenNI2_VERSION )
	FOREACH( VPART MAJOR MINOR MAINTENANCE )
		FOREACH( VLINE ${openni2_version_str} )
			IF( VLINE MATCHES "^#[\t ]*define[\t ]+ONI_VERSION_${VPART}[\t ]+([0-9]+)$" )
				SET( OpenNI2_VERSION_PART "${CMAKE_MATCH_1}" )
				IF( OpenNI2_VERSION )
					SET( OpenNI2_VERSION "${OpenNI2_VERSION}.${OpenNI2_VERSION_PART}" )
				ELSE()
					SET( OpenNI2_VERSION "${OpenNI2_VERSION_PART}" )
				ENDIF()
				UNSET( OpenNI2_VERSION_PART )
			ENDIF()
		ENDFOREACH()
	ENDFOREACH()
ENDIF()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
SET( OpenNI2_PROCESS_INCLUDES ${OpenNI2_INCLUDE_DIR} ${XnLib_INCLUDE_DIRS} ${LIBUSB_1_INCLUDE_DIRS} )
SET( OpenNI2_PROCESS_LIBS ${OpenNI2_LIBRARY} ${XnLib_LIBRARIES} ${LIBUSB_1_LIBRARIES} )
LIBFIND_PROCESS( OpenNI2 )

# Set IMPORTED Targets
IF( OpenNI2_FOUND )
	IF( NOT TARGET OpenNI2::OpenNI2 )
		ADD_LIBRARY( OpenNI2::OpenNI2 ${LIB_TYPE} IMPORTED )
		SET_TARGET_PROPERTIES( OpenNI2::OpenNI2 PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${OpenNI2_INCLUDE_DIRS}"
			VERSION "${OpenNI2_VERSION}" )

		IF( OpenNI2_LIBRARY_RELEASE )
			SET_PROPERTY( TARGET OpenNI2::OpenNI2 APPEND PROPERTY
				IMPORTED_CONFIGURATIONS RELEASE )
			SET_TARGET_PROPERTIES( OpenNI2::OpenNI2 PROPERTIES
				IMPORTED_LOCATION_RELEASE "${OpenNI2_LIBRARY_RELEASE}" )
		ENDIF()

		IF( OpenNI2_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET OpenNI2::OpenNI2 APPEND PROPERTY
				IMPORTED_CONFIGURATIONS DEBUG )
			SET_TARGET_PROPERTIES( OpenNI2::OpenNI2 PROPERTIES
				IMPORTED_LOCATION_DEBUG "${OpenNI2_LIBRARY_DEBUG}" )
		ENDIF()

		IF( NOT OpenNI2_LIBRARY_RELEASE AND NOT OpenNI2_LIBRARY_DEBUG )
			SET_PROPERTY( TARGET OpenNI2::OpenNI2 APPEND PROPERTY
				IMPORTED_LOCATION "${OpenNI2_LIBRARY}" )
		ENDIF()
	ENDIF()
	
	MESSAGE( STATUS "  Include Dirs: ${OpenNI2_INCLUDE_DIRS}" )
	MESSAGE( STATUS "  Libraries: ${OpenNI2_LIBRARIES}" )
	IF( OpenNI2_DEFINITIONS )
		MESSAGE( STATUS "  Definitions: ${OpenNI2_DEFINITIONS}" )
	ENDIF()
ENDIF()
