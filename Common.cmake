#-----------------------------------------------------------------------------
# Update CMake module path
#-----------------------------------------------------------------------------
LIST( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/CMake )


#-----------------------------------------------------------------------------
# Initialize compiler
#-----------------------------------------------------------------------------
ENABLE_LANGUAGE( C )
ENABLE_LANGUAGE( CXX )


#-----------------------------------------------------------------------------
# cmake includes
#-----------------------------------------------------------------------------
INCLUDE( CMakeDependentOption )
INCLUDE( ExternalProjectDependency )

#-----------------------------------------------------------------------------
# Set a default build type if none was specified
#-----------------------------------------------------------------------------
IF( NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES )
	MESSAGE( STATUS "Setting build type to 'Release' as none was specified." )
	SET( CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE )
	# Set the possible values of build type for cmake-gui
	SET_PROPERTY( CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo" )
ENDIF()


#-----------------------------------------------------------------------------
# Build option(s)
#-----------------------------------------------------------------------------
SET( BUILD_SHARED_LIBS OFF )


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
IF( CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64" )
	IF( NOT "${CMAKE_CXX_FLAGS}" MATCHES "-fPIC" )
		SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC" )
	ENDIF()
	IF( NOT "${CMAKE_C_FLAGS}" MATCHES "-fPIC" )
		SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC" )
	ENDIF()
ENDIF()

#-----------------------------------------------------------------------------
# Augment compiler flags
#-----------------------------------------------------------------------------

# On systems that support dynamic linking, this prevents linking with the
# shared libraries. On other systems, this option has no effect.  
IF( NOT "${CMAKE_CXX_FLAGS}" MATCHES "-static" )
	SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static" )
ENDIF()
IF( NOT "${CMAKE_C_FLAGS}" MATCHES "-static" )
	SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static" )
ENDIF()

# On systems that provide libgcc as a shared library, these options force
# the use of either the shared or static version, respectively. If no
# shared version of libgcc was built when the compiler was configured,
# these options have no effect.
IF( NOT "${CMAKE_CXX_FLAGS}" MATCHES "-static-libstdc\\+\\+" )
	SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++" )
ENDIF()
IF( NOT "${CMAKE_C_FLAGS}" MATCHES "-static-libgcc" )
	SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static-libgcc" )
ENDIF()

MARK_AS_SUPERBUILD(
	VARS
		MAKECOMMAND:STRING
		CMAKE_SKIP_RPATH:BOOL
		BUILD_SHARED_LIBS:BOOL
		CMAKE_MODULE_PATH:PATH
		CMAKE_BUILD_TYPE:STRING
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

