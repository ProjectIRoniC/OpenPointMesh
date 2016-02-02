#-----------------------------------------------------------------------------
# Update CMake module path
#------------------------------------------------------------------------------
LIST( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/CMake )

#-----------------------------------------------------------------------------
# ENABLE_LANGUAGE(C)	# not sure if this is needed
ENABLE_LANGUAGE( CXX )

INCLUDE( CMakeDependentOption )
INCLUDE( ExternalProjectDependency )

OPTION( ${PRIMARY_PROJECT_NAME}_INSTALL_DEVELOPMENT "Install development support include and libraries for external packages." OFF )
MARK_AS_ADVANCED( ${PRIMARY_PROJECT_NAME}_INSTALL_DEVELOPMENT )

## set( ITK_VERSION_MAJOR 4 CACHE STRING "Choose the expected ITK major version to build, only version 4 allowed." )	# don't think this is needed
## set_property( CACHE ITK_VERSION_MAJOR PROPERTY STRINGS "4" )		# don't think this is needed

#-----------------------------------------------------------------------------
# Set a default build type if none was specified
IF( NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES )
	MESSAGE( STATUS "Setting build type to 'Release' as none was specified." )
	SET( CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE )
	# Set the possible values of build type for cmake-gui
	SET_PROPERTY( CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo" )
ENDIF()

#-----------------------------------------------------------------------------
# Build option(s)
#-----------------------------------------------------------------------------
# OPTION(USE_BRAINSFit                      "Build BRAINSFit"                      ON)
# OPTION(USE_BRAINSSnapShotWriter           "Build BRAINSSnapShotWriter"           ON)
# IF( NOT USE_ANTS )
# 	OPTION(USE_ANTS                           "Build ANTs"                           ON)
# ENDIF()

OPTION( ${PRIMARY_PROJECT_NAME}_USE_QT "Find and use Qt with VTK to build GUI Tools" OFF )
MARK_AS_ADVANCED( ${PRIMARY_PROJECT_NAME}_USE_QT )

IF( ${PRIMARY_PROJECT_NAME}_USE_QT )
	IF( NOT QT4_FOUND )
		FIND_PACKAGE( Qt4 4.6 COMPONENTS QtCore QtGui QtNetwork QtXml REQUIRED )
		INCLUDE( ${QT_USE_FILE} )
	ENDIF()
ENDIF()

CMAKE_DEPENDENT_OPTION( ${PRIMARY_PROJECT_NAME}_USE_PYTHONQT "Use python with QT" OFF "${PRIMARY_PROJECT_NAME}_USE_QT" OFF )

IF( ${PRIMARY_PROJECT_NAME}_USE_PYTHONQT ) ## This is to force configuration of python early.
  ## NIPYPE is not stable under python 2.6, so require 2.7 when using autoworkup
  ## Enthought Canopy or anaconda are convenient ways to install python 2.7 on linux
  ## or the other option is the free version of Anaconda from https://store.continuum.io/
	SET( REQUIRED_PYTHON_VERSION 2.7 )
	IF( APPLE )
		SET( PYTHON_EXECUTABLE
			/System/Library/Frameworks/Python.framework/Versions/${REQUIRED_PYTHON_VERSION}/bin/python2.7
			CACHE FILEPATH "The apple specified python version" )
		SET( PYTHON_LIBRARY
			/System/Library/Frameworks/Python.framework/Versions/${REQUIRED_PYTHON_VERSION}/lib/libpython2.7.dylib
			CACHE FILEPATH "The apple specified python shared library" )
		SET( PYTHON_INCLUDE_DIR
			/System/Library/Frameworks/Python.framework/Versions/${REQUIRED_PYTHON_VERSION}/include/python2.7
			CACHE PATH "The apple specified python headers" )
	ELSE()
		IF( NOT EXISTS ${PYTHON_EXECUTABLE} )
			FIND_PACKAGE( PythonInterp ${REQUIRED_PYTHON_VERSION} REQUIRED )
		ENDIF()

		IF( NOT EXISTS ${PYTHON_LIBRARY} )
			MESSAGE( STATUS "Found PythonInterp version ${PYTHON_VERSION_STRING}" )
			FIND_PACKAGE( PythonLibs ${PYTHON_VERSION_STRING} EXACT REQUIRED )
		ENDIF()
	ENDIF()

MARK_AS_SUPERBUILD(
	VARS
		PYTHON_EXECUTABLE:FILEPATH
		PYTHON_LIBRARY:FILEPATH
		PYTHON_INCLUDE_DIR:PATH
	ALL_PROJECTS
	)
ENDIF()

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
IF( NOT COMMAND SETIFEMPTY )
	MACRO( SETIFEMPTY )
		SET( KEY ${ARGV0} )
		SET( VALUE ${ARGV1} )
		IF( NOT ${KEY} )
			SET( ${ARGV} )
		ENDIF()
	ENDMACRO()
ENDIF()

#-----------------------------------------------------------------------------
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin )
SET( CMAKE_BUNDLE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin )

#-------------------------------------------------------------------------
# SET( BRAINSTools_CLI_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )
# SET( BRAINSTools_CLI_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY} )
# SET( BRAINSTools_CLI_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} )

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
# Add needed flag for gnu on linux like enviroments to build static common libs
# suitable for linking with shared object libs.
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
#INCLUDE( ITKSetStandardCompilerFlags )

#------------------------------------------------------------------------
# Check for clang -- c++11 necessary for boost
#------------------------------------------------------------------------
IF( "${CMAKE_CXX_COMPILER}${CMAKE_CXX_COMPILER_ARG1}" MATCHES ".*clang.*" )
	SET( CMAKE_COMPILER_IS_CLANGXX ON CACHE BOOL "compiler is Clang" )
ENDIF()

SET( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} ${ITK_REQUIRED_C_FLAGS}" )
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ITK_REQUIRED_CXX_FLAGS}" )
SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ITK_REQUIRED_LINK_FLAGS}" )
SET( CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${ITK_REQUIRED_LINK_FLAGS}" )
SET( CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${ITK_REQUIRED_LINK_FLAGS}" )


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
#
# SimpleITK has large internal libraries, which take an extremely long
# time to link on windows when they are static. Creating shared
# SimpleITK internal libraries can reduce linking time. Also the size
# of the debug libraries are monstrous. Using shared libraries for
# debug, reduce disc requirements, and can improve linking
# times. However, these shared libraries take longer to load than the
# monolithic target from static libraries.
#
# SET( ${PRIMARY_PROJECT_NAME}_USE_SimpleITK_SHARED_DEFAULT OFF )
# STRING( TOUPPER "${CMAKE_BUILD_TYPE}" _CMAKE_BUILD_TYPE )
# IF( MSVC OR _CMAKE_BUILD_TYPE MATCHES "DEBUG" )
# 	SET( ${PRIMARY_PROJECT_NAME}_USE_SimpleITK_SHARED_DEFAULT ON )
# ENDIF()
# CMAKE_DEPENDENT_OPTION( ${PRIMARY_PROJECT_NAME}_USE_SimpleITK_SHARED "Build SimpleITK with shared libraries. Reduces linking time, increases run-time load time." ${${PRIMARY_PROJECT_NAME}_USE_SimpleITK_SHARED_DEFAULT} "${PRIMARY_PROJECT_NAME}_USE_SimpleITK" OFF )
# MARK_AS_SUPERBUILD( ${PRIMARY_PROJECT_NAME}_USE_SimpleITK_SHARED )

# TODO: figure out what this Slicer rigamarole is actually supposed to be doing
#-----------------------------------------------------------------------------
# ${PRIMARY_PROJECT_NAME} install directories
#-----------------------------------------------------------------------------
SET( ${PRIMARY_PROJECT_NAME}_INSTALL_ROOT "./" )
SET( ${PRIMARY_PROJECT_NAME}_BUNDLE_LOCATION "${${PRIMARY_PROJECT_NAME}_MAIN_PROJECT_APPLICATION_NAME}.app/Contents" )
# NOTE: Make sure to update vtk${PRIMARY_PROJECT_NAME}ApplicationLogic::IsEmbeddedModule if
#       the following variables are changed.
SET( ${PRIMARY_PROJECT_NAME}_EXTENSIONS_DIRBASENAME "Extensions" )
SET( ${PRIMARY_PROJECT_NAME}_EXTENSIONS_DIRNAME "${${PRIMARY_PROJECT_NAME}_EXTENSIONS_DIRBASENAME}-${${PRIMARY_PROJECT_NAME}_WC_REVISION}" )
IF( APPLE )
	SET( ${PRIMARY_PROJECT_NAME}_INSTALL_ROOT "${${PRIMARY_PROJECT_NAME}_BUNDLE_LOCATION}/" ) # Set to create Bundle
ENDIF()

SET( ${PRIMARY_PROJECT_NAME}_INSTALL_BIN_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_BIN_DIR}" )
SET( ${PRIMARY_PROJECT_NAME}_INSTALL_LIB_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_LIB_DIR}" )
SET( ${PRIMARY_PROJECT_NAME}_INSTALL_INCLUDE_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_INCLUDE_DIR}" )
SET( ${PRIMARY_PROJECT_NAME}_INSTALL_SHARE_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_SHARE_DIR}" )
# SET( ${PRIMARY_PROJECT_NAME}_INSTALL_ITKFACTORIES_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_LIB_DIR}/ITKFactories" )
# SET( ${PRIMARY_PROJECT_NAME}_INSTALL_QM_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_QM_DIR}" )

IF( ${PRIMARY_PROJECT_NAME}_BUILD_CLI_SUPPORT )
	SET( ${PRIMARY_PROJECT_NAME}_INSTALL_CLIMODULES_BIN_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_CLIMODULES_BIN_DIR}" )
	SET( ${PRIMARY_PROJECT_NAME}_INSTALL_CLIMODULES_LIB_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_CLIMODULES_LIB_DIR}" )
	SET( ${PRIMARY_PROJECT_NAME}_INSTALL_CLIMODULES_SHARE_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_CLIMODULES_SHARE_DIR}" )
ENDIF()

