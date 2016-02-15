#-----------------------------------------------------------------------------
# Update CMake module path
#-----------------------------------------------------------------------------
LIST( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/CMake )


#-----------------------------------------------------------------------------
# Initialize compiler
#-----------------------------------------------------------------------------
ENABLE_LANGUAGE( C )
ENABLE_LANGUAGE( CXX )

MESSAGE( STATUS "CMake Information" )
MESSAGE( STATUS "-- Generator:        ${CMAKE_GENERATOR}" )
MESSAGE( STATUS "-- Make Program:     ${CMAKE_MAKE_PROGRAM}" )
MESSAGE( STATUS "-- C Compiler ID:    ${CMAKE_C_COMPILER_ID}" )
MESSAGE( STATUS "-- C Compiler:       ${CMAKE_C_COMPILER}" )
MESSAGE( STATUS "-- CXX Compiler ID:  ${CMAKE_CXX_COMPILER_ID}" )
MESSAGE( STATUS "-- CXX Compiler:     ${CMAKE_CXX_COMPILER}" )


#-----------------------------------------------------------------------------
# cmake includes
#-----------------------------------------------------------------------------
INCLUDE( CMakeDependentOption )
#INCLUDE( ExternalProjectDependency )


#-----------------------------------------------------------------------------
# Set a default build type if none was specified
#-----------------------------------------------------------------------------
IF( NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES )
	MESSAGE( STATUS "Setting build type to 'Release' as none was specified." )
	SET( CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE )
	# Set the possible values of build type for cmake-gui
	SET_PROPERTY( CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo" )
ENDIF()

# Set make command for external projects not using cmake
#IF( WIN32 )
#	SET( ${PRIMARY_PROJECT_NAME}_NONCMAKE_BUILD_COMMAND "mingw32-make")
#ELSE()
#	SET( ${PRIMARY_PROJECT_NAME}_NONCMAKE_BUILD_COMMAND "make")
#ENDIF()


#-----------------------------------------------------------------------------
# Build option(s)
#-----------------------------------------------------------------------------




#-----------------------------------------------------------------------------
# Platform check
#-----------------------------------------------------------------------------
SET( PLATFORM_CHECK true )
IF( PLATFORM_CHECK )
	# See CMake/Modules/Platform/Darwin.cmake)
	#   6.x == Mac OSX 10.2 (Jaguar)
	#   7.x == Mac OSX 10.3 (Panther)
	#   8.x == Mac OSX 10.4 (Tiger)
	#   9.x == Mac OSX 10.5 (Leopard)
	#  10.x == Mac OSX 10.6 (Snow Leopard)
	IF( DARWIN_MAJOR_VERSION LESS "9" )
		MESSAGE( FATAL_ERROR "Only Mac OSX >= 10.5 are supported !" )
	ENDIF()
ENDIF()


#-----------------------------------------------------------------------------
#IF( NOT COMMAND SETIFEMPTY )
#	MACRO( SETIFEMPTY )
#		SET( KEY ${ARGV0} )
#		SET( VALUE ${ARGV1} )
#		IF( NOT ${KEY} )
#			SET( ${ARGV} )
#		ENDIF()
#	ENDMACRO()
#ENDIF()


#-----------------------------------------------------------------------------
# Set cmake output directories
#-----------------------------------------------------------------------------
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin )
SET( CMAKE_BUNDLE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin )

MARK_AS_SUPERBUILD(
	VARS
		CMAKE_LIBRARY_OUTPUT_DIRECTORY:PATH
		CMAKE_ARCHIVE_OUTPUT_DIRECTORY:PATH
		CMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH
		CMAKE_BUNDLE_OUTPUT_DIRECTORY:PATH
		CMAKE_INSTALL_RUNTIME_DESTINATION:PATH
		CMAKE_INSTALL_LIBRARY_DESTINATION:PATH
		CMAKE_INSTALL_ARCHIVE_DESTINATION:PATH
		CMAKE_BUNDLE_OUTPUT_DESTINATION:PATH
	ALL_PROJECTS
)


#-----------------------------------------------------------------------------
# Add needed flag for gnu on linux like enviroments to build static common
# libs suitable for linking with shared object libs.
#-----------------------------------------------------------------------------
IF( CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" )
	IF( NOT "${CMAKE_CXX_FLAGS}" MATCHES "-fPIC" )
		SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC" )
	ENDIF()
	IF( NOT "${CMAKE_C_FLAGS}" MATCHES "-fPIC" )
		SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC" )
	ENDIF()
ENDIF()


#-------------------------------------------------------------------------
# Augment compiler flags
#-------------------------------------------------------------------------


MARK_AS_SUPERBUILD(
	VARS
		MAKECOMMAND:STRING
		CMAKE_SKIP_RPATH:BOOL
		BUILD_SHARED_LIBS:BOOL
		CMAKE_MODULE_PATH:PATH
		CMAKE_BUILD_TYPE:STRING
		# BUILD_SHARED_LIBS:BOOL
		CMAKE_INCLUDE_DIRECTORIES_BEFORE:BOOL
		CMAKE_CXX_COMPILER:PATH
		CMAKE_CXX_FLAGS:STRING
		CMAKE_CXX_FLAGS_DEBUG:STRING
		CMAKE_CXX_FLAGS_MINSIZEREL:STRING
		CMAKE_CXX_FLAGS_RELEASE:STRING
		CMAKE_CXX_FLAGS_RELWITHDEBINFO:STRING
		CMAKE_C_COMPILER:PATH
		CMAKE_C_FLAGS:STRING
		CMAKE_C_FLAGS_DEBUG:STRING
		CMAKE_C_FLAGS_MINSIZEREL:STRING
		CMAKE_C_FLAGS_RELEASE:STRING
		CMAKE_C_FLAGS_RELWITHDEBINFO:STRING
		CMAKE_EXE_LINKER_FLAGS:STRING
		CMAKE_EXE_LINKER_FLAGS_DEBUG:STRING
		CMAKE_EXE_LINKER_FLAGS_MINSIZEREL:STRING
		CMAKE_EXE_LINKER_FLAGS_RELEASE:STRING
		CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO:STRING
		CMAKE_MODULE_LINKER_FLAGS:STRING
		CMAKE_MODULE_LINKER_FLAGS_DEBUG:STRING
		CMAKE_MODULE_LINKER_FLAGS_MINSIZEREL:STRING
		CMAKE_MODULE_LINKER_FLAGS_RELEASE:STRING
		CMAKE_MODULE_LINKER_FLAGS_RELWITHDEBINFO:STRING
		CMAKE_SHARED_LINKER_FLAGS:STRING
		CMAKE_SHARED_LINKER_FLAGS_DEBUG:STRING
		CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL:STRING
		CMAKE_SHARED_LINKER_FLAGS_RELEASE:STRING
		CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO:STRING
		CMAKE_GENERATOR:STRING
		CMAKE_EXTRA_GENERATOR:STRING
		CMAKE_EXPORT_COMPILE_COMMANDS:BOOL
		CMAKE_INSTALL_PREFIX:PATH
		CTEST_NEW_FORMAT:BOOL
		MEMORYCHECK_COMMAND_OPTIONS:STRING
		MEMORYCHECK_COMMAND:PATH
		CMAKE_SHARED_LINKER_FLAGS:STRING
		CMAKE_EXE_LINKER_FLAGS:STRING
		CMAKE_MODULE_LINKER_FLAGS:STRING
		SITE:STRING
		BUILDNAME:STRING
	ALL_PROJECTS
)


#-----------------------------------------------------------------------------
# ${PRIMARY_PROJECT_NAME} install directories
#-----------------------------------------------------------------------------
SET( ${PRIMARY_PROJECT_NAME}_INSTALL_ROOT "./" )
SET( ${PRIMARY_PROJECT_NAME}_BUNDLE_LOCATION "${${PRIMARY_PROJECT_NAME}_MAIN_PROJECT_APPLICATION_NAME}.app/Contents" )

SET( ${PRIMARY_PROJECT_NAME}_EXTENSIONS_DIRBASENAME "Extensions" )
SET( ${PRIMARY_PROJECT_NAME}_EXTENSIONS_DIRNAME "${${PRIMARY_PROJECT_NAME}_EXTENSIONS_DIRBASENAME}-${${PRIMARY_PROJECT_NAME}_WC_REVISION}" )
IF( APPLE )
	SET( ${PRIMARY_PROJECT_NAME}_INSTALL_ROOT "${${PRIMARY_PROJECT_NAME}_BUNDLE_LOCATION}/" ) # Set to create Bundle
ENDIF()

SET( ${PRIMARY_PROJECT_NAME}_INSTALL_BIN_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_BIN_DIR}" )
SET( ${PRIMARY_PROJECT_NAME}_INSTALL_LIB_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_LIB_DIR}" )
SET( ${PRIMARY_PROJECT_NAME}_INSTALL_INCLUDE_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_INCLUDE_DIR}" )
SET( ${PRIMARY_PROJECT_NAME}_INSTALL_SHARE_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_SHARE_DIR}" )

IF( ${PRIMARY_PROJECT_NAME}_BUILD_CLI_SUPPORT )
	SET( ${PRIMARY_PROJECT_NAME}_INSTALL_CLIMODULES_BIN_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_CLIMODULES_BIN_DIR}" )
	SET( ${PRIMARY_PROJECT_NAME}_INSTALL_CLIMODULES_LIB_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_CLIMODULES_LIB_DIR}" )
	SET( ${PRIMARY_PROJECT_NAME}_INSTALL_CLIMODULES_SHARE_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_CLIMODULES_SHARE_DIR}" )
ENDIF()

