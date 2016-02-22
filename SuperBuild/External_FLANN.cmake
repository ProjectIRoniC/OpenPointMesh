# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName FLANN ) # The find_package known name
SET( proj        FLANN ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "1.8.4" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES "" )

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

### --- Project specific additions here
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_CMAKE_OPTIONS
	# CMake Build ARGS
	-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
	-DCMAKE_CXX_FLAGS:STRING=${EP_COMMON_CXX_FLAGS}
	-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
	-DCMAKE_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
	-DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
	-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
	-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
	# FLANN ARGS
	-DBUILD_MATLAB_BINDINGS:BOOL=OFF
	-DBUILD_EXAMPLES:BOOL=OFF
	-DBUILD_TESTS:BOOL=OFF
	-DBUILD_DOC:BOOL=OFF
)

# The commits on github look useful but there are few release versions on github
# so we are going to use master
SET( ${proj}_REPOSITORY "${git_protocol}://github.com/mariusmuja/flann.git" )
SET( ${proj}_GIT_TAG master )
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	GIT_REPOSITORY ${${proj}_REPOSITORY}
	GIT_TAG ${${proj}_GIT_TAG}
	UPDATE_COMMAND ""	# we are skipping update so we don't have to build every time
	SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj}
	BINARY_DIR ${proj}-build
	LOG_CONFIGURE 0  # Wrap configure in script to ignore log output from dashboards
	LOG_BUILD     0  # Wrap build in script to to ignore log output from dashboards
	LOG_TEST      0  # Wrap test in script to to ignore log output from dashboards
	LOG_INSTALL   0  # Wrap install in script to to ignore log output from dashboards
	${cmakeversion_external_update} "${cmakeversion_external_update_value}"
	INSTALL_DIR ${${proj}_INSTALL_DIR}
	CMAKE_GENERATOR ${gen}
	CMAKE_ARGS -Wno-dev --no-warn-unused-cli
	CMAKE_CACHE_ARGS ${${proj}_CMAKE_OPTIONS}
	DEPENDS ${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( FLANN_DIR ${CMAKE_BINARY_DIR}/${proj}-install )
SET( FLANN_INCLUDE_DIR ${CMAKE_BINARY_DIR}/${proj}-install/include/flann )
SET( FLANN_LIBRARY_DIR ${CMAKE_BINARY_DIR}/${proj}-install/lib )

mark_as_superbuild(
	VARS
		FLANN_DIR:PATH
		FLANN_INCLUDE_DIR:PATH
		FLANN_LIBRARY_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "FLANN_DIR: ${FLANN_DIR}" )
ExternalProject_Message( ${proj} "FLANN_INCLUDE_DIR: ${FLANN_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "FLANN_LIBRARY_DIR: ${FLANN_LIBRARY_DIR}" )
### --- End binary information
