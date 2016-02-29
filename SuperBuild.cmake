##############################################################################
# SuperBuild script
#
# This file is responsible for setting up the Primary Project OpenPointMesh
# external project.  Setting up the Primary Project as an external project
# allows us to separate the SuperBuild build information from the Primary
# Project build information.  This file is where you should pass dependency
# information to the Primary Project cmake build script.
##############################################################################
#------------------------------------------------------------------------------
# ${PRIMARY_PROJECT_NAME} dependency list
#------------------------------------------------------------------------------
SET( extProjName ${PRIMARY_PROJECT_NAME} )
SET( proj        ${PRIMARY_PROJECT_NAME} )

# Set dependency list
SET( ${PRIMARY_PROJECT_NAME}_DEPENDENCIES
	PCL
	Qt
	VTK
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} DEPENDS_VAR ${PRIMARY_PROJECT_NAME}_DEPENDENCIES )


#------------------------------------------------------------------------------
# Configure and build ${PRIMARY_PROJECT_NAME}
#------------------------------------------------------------------------------
SET( proj ${PRIMARY_PROJECT_NAME} )
EXTERNALPROJECT_ADD( ${proj}
	${${proj}_EP_ARGS}
	DEPENDS ${${PRIMARY_PROJECT_NAME}_DEPENDENCIES}
	SOURCE_DIR	${${PRIMARY_PROJECT_NAME}_SOURCE_DIR}
	BINARY_DIR	${${PRIMARY_PROJECT_NAME}_BUILD_DIR}
	DOWNLOAD_COMMAND	""
	UPDATE_COMMAND		""
	INSTALL_COMMAND		""
	CMAKE_GENERATOR	${gen}
	CMAKE_ARGS		--no-warn-unused-cli    # HACK Only expected variables should be passed down.
	CMAKE_CACHE_ARGS
		-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
		-DCMAKE_CXX_FLAGS:STRING=${EP_COMMON_CXX_FLAGS}
		-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
		-DCMAKE_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
		-D${PRIMARY_PROJECT_NAME}_SUPERBUILD:BOOL=OFF    #NOTE: VERY IMPORTANT reprocess top level CMakeList.txt
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
