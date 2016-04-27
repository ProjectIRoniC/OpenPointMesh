##############################################################################
# Common variables and functionality for the Primary and Dependency Projects
#
# This file is responsible for adding common functionality, setup build
# options, and setup build flags for the primary project and dependency
# projects
##############################################################################
#-----------------------------------------------------------------------------
# Build option(s)
#-----------------------------------------------------------------------------
# CMake Options
OPTION( CMAKE_INCLUDE_DIRECTORIES_BEFORE "Set to prepend include directories" ON )
OPTION( CMAKE_POSITION_INDEPENDENT_CODE "Set to use Position Independent Code" ON )

# Library Options
OPTION( BUILD_SHARED_LIBS "Build Shared Libraries" OFF )
IF( APPLE )
	# OS X does not support static os libraries so we must use a shared library build
	SET( BUILD_SHARED_LIBS ON FORCE )
ENDIF()

OPTION( LINK_TIME_OPTIMIZATION "***** WARNING EXPERIMENTAL ***** Use Link Time Optimization (only affects Release and RelWithDebInfo builds)" OFF )

# Output Options
OPTION( EP_LOG_DOWNLOAD "Wrap External Projects download in script to log output" OFF )
OPTION( EP_LOG_UPDATE "Wrap External Projects update in script to log output" OFF )
OPTION( EP_LOG_CONFIGURE "Wrap External Projects configure in script to log output" OFF )
OPTION( EP_LOG_BUILD "Wrap External Projects build in script to log output" OFF )
OPTION( EP_LOG_TEST "Wrap External Projects test in script to log output" OFF )
OPTION( EP_LOG_INSTALL "Wrap External Projects install in script to log output" OFF )

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
# Common Settings
#-----------------------------------------------------------------------------
# Initialize compiler
ENABLE_LANGUAGE( C )
ENABLE_LANGUAGE( CXX )

# Set the c/cxx standard
IF( NOT CMAKE_C_STANDARD )
	SET( CMAKE_C_STANDARD 11 ) # Supported values are ``90``, ``99`` and ``11``
	SET( CMAKE_C_STANDARD_REQUIRED ON )
ENDIF()
IF( NOT CMAKE_CXX_STANDARD )
	SET( CMAKE_CXX_STANDARD 14 ) # Supported values are ``98``, ``11`` and ``14``
	SET( CMAKE_CXX_STANDARD_REQUIRED ON )
ENDIF()

# cmake includes
INCLUDE( CMakeDependentOption )
INCLUDE( ExternalProject )
INCLUDE( ExternalProjectDependency )
FIND_PACKAGE( PythonLibs 3 REQUIRED )
FIND_PACKAGE( OpenGL REQUIRED )
FIND_PACKAGE( GLUT REQUIRED )

# Add paths to external project helper scripts
SET( RUN_AUTOGENSH_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/SuperBuild/External_runautogensh.cmake )
SET( RUN_AUTORECONF_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/SuperBuild/External_runautoreconf.cmake )
SET( UPDATE_CONFIG_GUESS_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/SuperBuild/External_updateguess.cmake )

# Setup a source cache so projects can share downloaded sources
# By keeping this outside of the build tree, you can share one
# set of external source trees for multiple build trees
SET( SOURCE_DOWNLOAD_CACHE ${CMAKE_CURRENT_BINARY_DIR} CACHE PATH
    "The path for downloading external source directories" )
MARK_AS_ADVANCED( SOURCE_DOWNLOAD_CACHE )

# Set cmake output directories
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin )
SET( CMAKE_BUNDLE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin )
SET( CMAKE_INSTALL_LIBRARY_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/install/lib )
SET( CMAKE_INSTALL_ARCHIVE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/install/lib )
SET( CMAKE_INSTALL_RUNTIME_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/install/bin )
SET( CMAKE_BUNDLE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/install/bin )


#-----------------------------------------------------------------------------
# Setup RPATH
#-----------------------------------------------------------------------------
# disable automatic mac rpath settings
SET( CMAKE_MACOSX_RPATH ON )

# use, i.e. don't skip the full RPATH for the build tree
SET( CMAKE_SKIP_BUILD_RPATH OFF )

# when building, don't use the install RPATH already (but later on when installing)
SET( CMAKE_BUILD_WITH_INSTALL_RPATH OFF ) 

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET( CMAKE_INSTALL_RPATH_USE_LINK_PATH ON )


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
# Augment compiler flags
#-----------------------------------------------------------------------------
# Set BUILD_SHARED_LIBS option settings
IF( BUILD_SHARED_LIBS )
	IF( NOT APPLE ) # OS X does not support this flag
		# Produce a shared object which can then be linked with other objects to
		# form an executable. Not all systems support this option. For 
		# predictable results, you must also specify the same set of options used
		# for compilation (-fpic, -fPIC, or model suboptions) when you specify
		# this linker option.
		IF( NOT "${NONCMAKE_CXX_FLAGS}" MATCHES "-shared" )
			SET( NONCMAKE_CXX_FLAGS "${NONCMAKE_CXX_FLAGS} -shared" )
		ENDIF()
		IF( NOT "${NONCMAKE_C_FLAGS}" MATCHES "-shared" )
			SET( NONCMAKE_C_FLAGS "${NONCMAKE_C_FLAGS} -shared" )
		ENDIF()
	ENDIF()
ELSE()
	# On systems that support dynamic linking, this prevents linking with the
	# shared libraries. On other systems, this option has no effect.  
	IF( NOT "${NONCMAKE_CXX_FLAGS}" MATCHES "-static" )
		SET( NONCMAKE_CXX_FLAGS "${NONCMAKE_CXX_FLAGS} -static" )
	ENDIF()
	IF( NOT "${NONCMAKE_C_FLAGS}" MATCHES "-static" )
		SET( NONCMAKE_C_FLAGS "${NONCMAKE_C_FLAGS} -static" )
	ENDIF()

	# On systems that provide libgcc as a shared library, these options force
	# the use of either the shared or static version, respectively. If no
	# shared version of libgcc was built when the compiler was configured,
	# these options have no effect.
	IF( NOT "${CMAKE_EXE_LINKER_C_FLAGS}" MATCHES "-static-libstdc\\+\\+" )
		SET( CMAKE_EXE_LINKER_C_FLAGS "${CMAKE_EXE_LINKER_C_FLAGS} -static-libstdc++" )
	ENDIF()
	IF( NOT "${CMAKE_EXE_LINKER_CXX_FLAGS}" MATCHES "-static-libgcc" )
		SET( CMAKE_EXE_LINKER_CXX_FLAGS "${CMAKE_EXE_LINKER_CXX_FLAGS} -static-libgcc" )
	ENDIF()
ENDIF()

# Set LINK_TIME_OPTIMIZATION option settings
# This doesn't quite work on MSYS2 yet, there is currently an issue where
# the test compiler doesn't find the plugin needed to handle lto
IF( LINK_TIME_OPTIMIZATION AND NOT "${CMAKE_BUILD_TYPE}" MATCHES "Debug" AND NOT "${CMAKE_BUILD_TYPE}" MATCHES "RelWithDebInfo" )
	# This option runs the standard link-time optimizer. When invoked with
	# source code, it generates GIMPLE (one of GCC's internal representations)
	# and writes it to special ELF sections in the object file. When the
	# object files are linked together, all the function bodies are read from
	# these ELF sections and instantiated as if they had been part of the same
	# translation unit.
	IF( NOT "${CMAKE_C_FLAGS}" MATCHES "-flto" )
		SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -flto" )
	ENDIF()
	IF( NOT "${CMAKE_CXX_FLAGS}" MATCHES "-flto" )
		SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto" )
	ENDIF()
	
	# This option enables the extraction of object files with GIMPLE bytecode
	# out of library archives. This improves the quality of optimization by
	# exposing more code to the link-time optimizer. This information
	# specifies what symbols can be accessed externally (by non-LTO object or
	# during dynamic linking).
	IF( NOT "${CMAKE_C_FLAGS}" MATCHES "-fuse-linker-plugin" )
		SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fuse-linker-plugin" )
	ENDIF()
	IF( NOT "${CMAKE_CXX_FLAGS}" MATCHES "-fuse-linker-plugin" )
		SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-linker-plugin" )
	ENDIF()
		
	# For the 64bit target, the assembler includes a new -Wa,-mbig-obj option
	# to allow objects with up to 2**31 sections.
	# NOTE: This is needed to build FLANN, possibly others
	IF( NOT "${CMAKE_C_FLAGS}" MATCHES "-Wa,-mbig-obj" )
		SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wa,-mbig-obj" )
	ENDIF()
	IF( NOT "${CMAKE_CXX_FLAGS}" MATCHES "-Wa,-mbig-obj" )
		SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj" )
	ENDIF()
ENDIF()

# Set the C standard flags
IF( CMAKE_C_STANDARD EQUAL 11 )
	IF( NOT "${NONCMAKE_C_FLAGS}" MATCHES "-std=gnu11" )
		SET( NONCMAKE_C_FLAGS "${NONCMAKE_C_FLAGS} -std=gnu11" )
	ENDIF()
ELSEIF( CMAKE_C_STANDARD EQUAL 99 )
	IF( NOT "${NONCMAKE_C_FLAGS}" MATCHES "-std=gnu99" )
		SET( NONCMAKE_C_FLAGS "${NONCMAKE_C_FLAGS} -std=gnu99" )
	ENDIF()
ELSEIF( CMAKE_C_STANDARD EQUAL 90 )
	IF( NOT "${NONCMAKE_C_FLAGS}" MATCHES "-std=gnu90" )
		SET( NONCMAKE_C_FLAGS "${NONCMAKE_C_FLAGS} -std=gnu90" )
	ENDIF()
ENDIF()

# Set the CXX standard flags
IF( CMAKE_CXX_STANDARD EQUAL 14 )
	IF( NOT "${NONCMAKE_CXX_FLAGS}" MATCHES "-std=gnu\\+\\+14" )
		SET( NONCMAKE_CXX_FLAGS "${NONCMAKE_CXX_FLAGS} -std=gnu++14" )
	ENDIF()
ELSEIF( CMAKE_CXX_STANDARD EQUAL 11 )
	IF( NOT "${NONCMAKE_CXX_FLAGS}" MATCHES "-std=gnu\\+\\+11" )
		SET( NONCMAKE_CXX_FLAGS "${NONCMAKE_CXX_FLAGS} -std=gnu++11" )
	ENDIF()
ELSEIF( CMAKE_CXX_STANDARD EQUAL 99 )
	IF( NOT "${NONCMAKE_CXX_FLAGS}" MATCHES "-std=gnu\\+\\+99" )
		SET( NONCMAKE_CXX_FLAGS "${NONCMAKE_CXX_FLAGS} -std=gnu++99" )
	ENDIF()
ENDIF()


#-----------------------------------------------------------------------------
# Primary Project Settings
#-----------------------------------------------------------------------------
SET( ${PRIMARY_PROJECT_NAME}_BUILD_DIR ${PRIMARY_PROJECT_NAME}-build )
SET( ${PRIMARY_PROJECT_NAME}_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} )

SET( ${PRIMARY_PROJECT_NAME}_INSTALL_ROOT "${CMAKE_CURRENT_BINARY_DIR}/install" )
IF( APPLE )
	SET( ${PRIMARY_PROJECT_NAME}_BUNDLE_LOCATION "${PRIMARY_PROJECT_NAME}.app/Contents" )
	SET( ${PRIMARY_PROJECT_NAME}_INSTALL_ROOT "${CMAKE_CURRENT_BINARY_DIR}/install/${${PRIMARY_PROJECT_NAME}_BUNDLE_LOCATION}" )
ENDIF()

SET( ${PRIMARY_PROJECT_NAME}_BIN_DIR "bin" )
SET( ${PRIMARY_PROJECT_NAME}_LIB_DIR "lib" )
SET( ${PRIMARY_PROJECT_NAME}_INCLUDE_DIR "include" )
SET( ${PRIMARY_PROJECT_NAME}_SHARE_DIR "share" )

SET( ${PRIMARY_PROJECT_NAME}_INSTALL_BIN_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_BIN_DIR}" )
SET( ${PRIMARY_PROJECT_NAME}_INSTALL_LIB_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_LIB_DIR}" )
SET( ${PRIMARY_PROJECT_NAME}_INSTALL_INCLUDE_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_INCLUDE_DIR}" )
SET( ${PRIMARY_PROJECT_NAME}_INSTALL_SHARE_DIR "${${PRIMARY_PROJECT_NAME}_INSTALL_ROOT}${${PRIMARY_PROJECT_NAME}_SHARE_DIR}" )

# Add variables to Primary Project
MARK_AS_SUPERBUILD(
	VARS
		${PRIMARY_PROJECT_NAME}_BUILD_DIR:PATH
		${PRIMARY_PROJECT_NAME}_SOURCE_DIR:PATH
		${PRIMARY_PROJECT_NAME}_INSTALL_ROOT:PATH
		${PRIMARY_PROJECT_NAME}_BUNDLE_LOCATION:PATH
		${PRIMARY_PROJECT_NAME}_BIN_DIR:PATH
		${PRIMARY_PROJECT_NAME}_LIB_DIR:PATH
		${PRIMARY_PROJECT_NAME}_INCLUDE_DIR:PATH
		${PRIMARY_PROJECT_NAME}_SHARE_DIR:PATH
		${PRIMARY_PROJECT_NAME}_INSTALL_BIN_DIR:PATH
		${PRIMARY_PROJECT_NAME}_INSTALL_LIB_DIR:PATH
		${PRIMARY_PROJECT_NAME}_INSTALL_INCLUDE_DIR:PATH
		${PRIMARY_PROJECT_NAME}_INSTALL_SHARE_DIR:PATH
	${PRIMARY_PROJECT_NAME}
)

#-----------------------------------------------------------------------------
# External Project Settings
#-----------------------------------------------------------------------------
# Set common c/cxx flags for external projects
SET( EP_COMMON_C_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_INIT} -Wno-unused" )
SET( EP_COMMON_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_INIT} -Wno-unused" )
SET( EP_NONCMAKE_COMMON_C_FLAGS "${EP_COMMON_C_FLAGS} ${NONCMAKE_C_FLAGS}" )
SET( EP_NONCMAKE_COMMON_CXX_FLAGS "${EP_COMMON_CXX_FLAGS} ${NONCMAKE_CXX_FLAGS}" )

# Set linker flags
SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_C_FLAGS} ${CMAKE_EXE_LINKER_CXX_FLAGS}" )

# Set external project cmake args
SET( EP_CMAKE_ARGS "-Wno-dev --no-warn-unused-cli" )

# Add variables to all projects
MARK_AS_SUPERBUILD(
	VARS
		BUILD_SHARED_LIBS:BOOL
		BUILDNAME:STRING
		MAKECOMMAND:STRING
		CMAKE_SKIP_RPATH:BOOL
		CMAKE_MODULE_PATH:PATH
		CMAKE_BUILD_TYPE:STRING
		CMAKE_INCLUDE_DIRECTORIES_BEFORE:BOOL
		CMAKE_C_COMPILER:PATH
		CMAKE_C_FLAGS:STRING
		CMAKE_C_FLAGS_DEBUG:STRING
		CMAKE_C_FLAGS_MINSIZEREL:STRING
		CMAKE_C_FLAGS_RELEASE:STRING
		CMAKE_C_FLAGS_RELWITHDEBINFO:STRING
		CMAKE_C_STANDARD:STRING
		CMAKE_CXX_COMPILER:PATH
		CMAKE_CXX_FLAGS:STRING
		CMAKE_CXX_FLAGS_DEBUG:STRING
		CMAKE_CXX_FLAGS_MINSIZEREL:STRING
		CMAKE_CXX_FLAGS_RELEASE:STRING
		CMAKE_CXX_FLAGS_RELWITHDEBINFO:STRING
		CMAKE_CXX_STANDARD:STRING
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
		CMAKE_POSITION_INDEPENDENT_CODE:BOOL
		CMAKE_SHARED_LINKER_FLAGS:STRING
		CMAKE_SHARED_LINKER_FLAGS_DEBUG:STRING
		CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL:STRING
		CMAKE_SHARED_LINKER_FLAGS_RELEASE:STRING
		CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO:STRING
		CMAKE_GENERATOR:STRING
		CMAKE_EXTRA_GENERATOR:STRING
		CMAKE_EXPORT_COMPILE_COMMANDS:BOOL
		CMAKE_INSTALL_PREFIX:PATH
		CMAKE_LIBRARY_OUTPUT_DIRECTORY:PATH
		CMAKE_ARCHIVE_OUTPUT_DIRECTORY:PATH
		CMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH
		CMAKE_BUNDLE_OUTPUT_DIRECTORY:PATH
		CMAKE_INSTALL_LIBRARY_DIRECTORY:PATH
		CMAKE_INSTALL_ARCHIVE_DIRECTORY:PATH
		CMAKE_INSTALL_RUNTIME_DIRECTORY:PATH
		CMAKE_BUNDLE_OUTPUT_DIRECTORY:PATH
		CMAKE_MACOSX_RPATH:BOOL
		CMAKE_SKIP_BUILD_RPATH:BOOL
		CMAKE_BUILD_WITH_INSTALL_RPATH:BOOL
		CMAKE_INSTALL_RPATH_USE_LINK_PATH:BOOL
		CTEST_NEW_FORMAT:BOOL
		MEMORYCHECK_COMMAND_OPTIONS:STRING
		MEMORYCHECK_COMMAND:PATH
		EP_COMMON_C_FLAGS:STRING
		EP_COMMON_CXX_FLAGS:STRING
		EP_CMAKE_ARGS:STRING
		EP_NONCMAKE_COMMON_C_FLAGS:STRING
		EP_NONCMAKE_COMMON_CXX_FLAGS:STRING
		CMAKE_EXE_LINKER_C_FLAGS:STRING
		CMAKE_EXE_LINKER_CXX_FLAGS:STRING
		RUN_AUTOGENSH_SCRIPT:FILEPATH
		RUN_AUTORECONF_SCRIPT:FILEPATH
		UPDATE_CONFIG_GUESS_SCRIPT:FILEPATH
	ALL_PROJECTS
)


#-----------------------------------------------------------------------------
# Git protocol option
#-----------------------------------------------------------------------------
OPTION( ${CMAKE_PROJECT_NAME}_USE_GIT_PROTOCOL "If behind a firewall turn this off to use http instead." ON )
SET( git_protocol "git" )
IF( NOT ${CMAKE_PROJECT_NAME}_USE_GIT_PROTOCOL )
	SET( git_protocol "http" )
	# Verify that the global git config has been updated with the expected "insteadOf" option.
	FUNCTION( _check_for_required_git_config_insteadof base insteadof )
		EXECUTE_PROCESS(
			COMMAND ${GIT_EXECUTABLE} config --global --get "url.${base}.insteadof"
			OUTPUT_VARIABLE output
			OUTPUT_STRIP_TRAILING_WHITESPACE
			RESULT_VARIABLE error_code
			)
		IF( error_code OR NOT "${output}" STREQUAL "${insteadof}" )
			MESSAGE( FATAL_ERROR
"Since the ExternalProject modules doesn't provide a mechanism to customize the clone step by "
"adding 'git config' statement between the 'git checkout' and the 'submodule init', it is required "
"to manually update your global git config to successfully build ${CMAKE_PROJECT_NAME} with "
"option ${CMAKE_PROJECT_NAME}_USE_GIT_PROTOCOL set to FALSE. "
"See http://na-mic.org/Mantis/view.php?id=2731"
"\nYou could do so by running the command:\n"
"  ${GIT_EXECUTABLE} config --global url.\"${base}\".insteadOf \"${insteadof}\"\n")
		ENDIF()
	ENDFUNCTION()
ENDIF()

FIND_PACKAGE( Git REQUIRED )
