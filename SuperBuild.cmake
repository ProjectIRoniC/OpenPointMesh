#-----------------------------------------------------------------------------
ENABLE_LANGUAGE( C )	# not sure if this is needed
ENABLE_LANGUAGE( CXX )

#-----------------------------------------------------------------------------
# INCLUDE( SlicerMacroGetOperatingSystemArchitectureBitness )	# not sure if this is needed

#-----------------------------------------------------------------------------
# Where should the superbuild source files be downloaded to?
# By keeping this outside of the build tree, you can share one
# set of external source trees for multiple build trees
#-----------------------------------------------------------------------------
SET( SOURCE_DOWNLOAD_CACHE ${CMAKE_CURRENT_BINARY_DIR} CACHE PATH
    "The path for downloading external source directories" )
MARK_AS_ADVANCED( SOURCE_DOWNLOAD_CACHE )

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

CMAKE_DEPENDENT_OPTION( ${CMAKE_PROJECT_NAME}_USE_CTKAPPLAUNCHER "CTKAppLauncher used with python" ON
  "NOT ${CMAKE_PROJECT_NAME}_USE_SYSTEM_python" OFF )

FIND_PACKAGE( Git REQUIRED )

# I don't know who removed the Find_Package for QT, but it needs to be here
# in order to build VTK if ${PRIMARY_PROJECT_NAME}_USE_QT is set.
IF( ${PRIMARY_PROJECT_NAME}_USE_QT )
	CMAKE_DEPENDENT_OPTION(
		BUILD_DTIPrep "BUILD_DTIPrep option" OFF "${PRIMARY_PROJECT_NAME}_USE_QT" ON
	)
	SET( QT_DEPENDENT_PACKAGES ) # vv package can also be built but was causing problems
	IF( BUILD_DTIPrep )  # Do not build DTIPrep until it behave better. Hans 2015-01-30
		SET( QT_DEPENDENT_PACKAGES DTIPrep ) # vv package can also be built but was causing problems
	ENDIF()
	FIND_PACKAGE( Qt4 REQUIRED )
ELSE()
	SET( QT_DEPENDENT_PACKAGES "" )
ENDIF()

#-----------------------------------------------------------------------------
# Enable and setup External project global properties
#-----------------------------------------------------------------------------
INCLUDE( ExternalProject )
#INCLUDE( MacroCheckExternalProjectDependency )
#INCLUDE( MacroEmptyExternalProject )

# Compute -G arg for configuring external projects with the same CMake generator:
IF( CMAKE_EXTRA_GENERATOR )
	SET( gen "${CMAKE_EXTRA_GENERATOR} - ${CMAKE_GENERATOR}" )
ELSE()
	SET( gen "${CMAKE_GENERATOR}" )
ENDIF()


# With CMake 2.8.9 or later, the UPDATE_COMMAND is required for updates to occur.
# For earlier versions, we nullify the update state to prevent updates and
# undesirable rebuild.
OPTION( FORCE_EXTERNAL_BUILDS "Force rebuilding of external project (if they are updated)" ON )
IF( CMAKE_VERSION VERSION_LESS 2.8.9 OR NOT FORCE_EXTERNAL_BUILDS )
	SET( cmakeversion_external_update UPDATE_COMMAND )
	SET( cmakeversion_external_update_value "" )
ELSE()
	SET( cmakeversion_external_update LOG_UPDATE )
	SET( cmakeversion_external_update_value 1 )
ENDIF()

#-----------------------------------------------------------------------------
# Superbuild option(s)
#-----------------------------------------------------------------------------
# OPTION( BUILD_STYLE_UTILS "Build uncrustify, cppcheck, & KWStyle" OFF )
# CMAKE_DEPENDENT_OPTION(
#	USE_SYSTEM_Uncrustify "Use system Uncrustify program" OFF
#	"BUILD_STYLE_UTILS" OFF
#	)
# CMAKE_DEPENDENT_OPTION(
#	USE_SYSTEM_KWStyle "Use system KWStyle program" OFF
#	"BUILD_STYLE_UTILS" OFF
#	)
# CMAKE_DEPENDENT_OPTION(
#	USE_SYSTEM_Cppcheck "Use system Cppcheck program" OFF
#	"BUILD_STYLE_UTILS" OFF
#	)

SET( EXTERNAL_PROJECT_BUILD_TYPE "Release" CACHE STRING "Default build type for support libraries" )
SET_PROPERTY( CACHE EXTERNAL_PROJECT_BUILD_TYPE PROPERTY
	STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo" )

# OPTION( USE_SYSTEM_VTK "Build using an externally defined version of VTK" OFF )


#-----------------------------------------------------------------------------
# Common external projects CMake variables
#-----------------------------------------------------------------------------
SET( CMAKE_INCLUDE_DIRECTORIES_BEFORE OFF CACHE BOOL "Set default to prepend include directories." )

SET( CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Write compile_commands.json" )


IF( ${PRIMARY_PROJECT_NAME}_USE_QT )
	MARK_AS_SUPERBUILD(
		VARS
			${PRIMARY_PROJECT_NAME}_USE_QT:BOOL
			QT_QMAKE_EXECUTABLE:PATH
			QT_MOC_EXECUTABLE:PATH
			QT_UIC_EXECUTABLE:PATH
		ALL_PROJECTS
	)
ENDIF()
MARK_AS_SUPERBUILD( ${PRIMARY_PROJECT_NAME}_USE_QT )

SET( extProjName ${PRIMARY_PROJECT_NAME} )
SET( proj        ${PRIMARY_PROJECT_NAME} )


#-----------------------------------------------------------------------------
# Set CMake OSX variable to pass down the external projects
#-----------------------------------------------------------------------------
# IF( APPLE )
#	MARK_AS_SUPERBUILD(
#		VARS
#			CMAKE_OSX_ARCHITECTURES:STRING
#			CMAKE_OSX_SYSROOT:PATH
#			CMAKE_OSX_DEPLOYMENT_TARGET:STRING
#		ALL_PROJECTS
#		)
# ENDIF()

SET( ${PRIMARY_PROJECT_NAME}_CLI_RUNTIME_DESTINATION  bin )
SET( ${PRIMARY_PROJECT_NAME}_CLI_LIBRARY_DESTINATION  lib )
SET( ${PRIMARY_PROJECT_NAME}_CLI_ARCHIVE_DESTINATION  lib )
SET( ${PRIMARY_PROJECT_NAME}_CLI_INSTALL_RUNTIME_DESTINATION  bin )
SET( ${PRIMARY_PROJECT_NAME}_CLI_INSTALL_LIBRARY_DESTINATION  lib )
SET( ${PRIMARY_PROJECT_NAME}_CLI_INSTALL_ARCHIVE_DESTINATION  lib )
#-----------------------------------------------------------------------------
# Add external project CMake args
#-----------------------------------------------------------------------------

MARK_AS_SUPERBUILD(
	VARS
#		BUILD_EXAMPLES:BOOL
#		BUILD_TESTING:BOOL
#		ITK_VERSION_MAJOR:STRING
#		ITK_DIR:PATH

		${PRIMARY_PROJECT_NAME}_CLI_LIBRARY_OUTPUT_DIRECTORY:PATH
		${PRIMARY_PROJECT_NAME}_CLI_ARCHIVE_OUTPUT_DIRECTORY:PATH
		${PRIMARY_PROJECT_NAME}_CLI_RUNTIME_OUTPUT_DIRECTORY:PATH
		${PRIMARY_PROJECT_NAME}_CLI_INSTALL_LIBRARY_DESTINATION:PATH
		${PRIMARY_PROJECT_NAME}_CLI_INSTALL_ARCHIVE_DESTINATION:PATH
		${PRIMARY_PROJECT_NAME}_CLI_INSTALL_RUNTIME_DESTINATION:PATH

		INSTALL_RUNTIME_DESTINATION:STRING
		INSTALL_LIBRARY_DESTINATION:STRING
		INSTALL_ARCHIVE_DESTINATION:STRING
	ALL_PROJECTS
)


STRING( REPLACE ";" "^" ${CMAKE_PROJECT_NAME}_SUPERBUILD_EP_VARNAMES "${${CMAKE_PROJECT_NAME}_SUPERBUILD_EP_VARNAMES}" )

#------------------------------------------------------------------------------
# ${PRIMARY_PROJECT_NAME} dependency list
#------------------------------------------------------------------------------

## for i in SuperBuild/*; do  echo $i |sed 's/.*External_\([a-zA-Z]*\).*/\1/g'|fgrep -v cmake|fgrep -v Template; done|sort -u
SET( ${PRIMARY_PROJECT_NAME}_DEPENDENCIES
	PCL
	# Qt4
	# VTK
#	JPEG
	${QT_DEPENDENT_PACKAGES}
)

# IF( BUILD_STYLE_UTILS )
#	LIST( APPEND ${PRIMARY_PROJECT_NAME}_DEPENDENCIES Cppcheck KWStyle ) #Uncrustify)
# ENDIF()


#-----------------------------------------------------------------------------
# Enable and setup External project global properties
#-----------------------------------------------------------------------------

SET( ep_common_c_flags "${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_INIT} ${ADDITIONAL_C_FLAGS}" )
SET( ep_common_cxx_flags "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_INIT} ${ADDITIONAL_CXX_FLAGS}" )

EXTERNALPROJECT_INCLUDE_DEPENDENCIES( ${proj} DEPENDS_VAR ${PRIMARY_PROJECT_NAME}_DEPENDENCIES )


#------------------------------------------------------------------------------
# Configure and build ${PROJECT_NAME}
#------------------------------------------------------------------------------
SET( proj ${PRIMARY_PROJECT_NAME} )
EXTERNALPROJECT_ADD( ${proj}
	${${proj}_EP_ARGS}
	DEPENDS ${${PRIMARY_PROJECT_NAME}_DEPENDENCIES}
	SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
	BINARY_DIR ${PRIMARY_PROJECT_NAME}-build
	DOWNLOAD_COMMAND ""
	UPDATE_COMMAND ""
	CMAKE_GENERATOR ${gen}
	CMAKE_ARGS
		--no-warn-unused-cli    # HACK Only expected variables should be passed down.
	CMAKE_CACHE_ARGS
			-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
			-DCMAKE_CXX_FLAGS:STRING=${ep_common_cxx_flags}
			-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
			-DCMAKE_C_FLAGS:STRING=${ep_common_c_flags}
		-D${PRIMARY_PROJECT_NAME}_SUPERBUILD:BOOL=OFF    #NOTE: VERY IMPORTANT reprocess top level CMakeList.txt
	INSTALL_COMMAND ""
)

# This custom external project step forces the build and later
# steps to run whenever a top level build is done...
EXTERNALPROJECT_ADD_STEP( ${proj} forcebuild
	COMMAND ${CMAKE_COMMAND} -E remove
		${CMAKE_CURRENT_BINARY_DIR}/${proj}-prefix/src/${proj}-stamp/${proj}-build
	COMMENT "Forcing build step for '${proj}'"
	DEPENDEES build
	ALWAYS 1
)
