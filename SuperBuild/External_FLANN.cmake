# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName FLANN ) # The find_package known name
SET( proj        FLANN ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES "" )

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-build )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( ${proj}_CMAKE_OPTIONS
	# CMake Build ARGS
	-DCMAKE_INCLUDE_DIRECTORIES_BEFORE:BOOL=${CMAKE_INCLUDE_DIRECTORIES_BEFORE}
	-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
	-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
	-DCMAKE_SKIP_BUILD_RPATH:BOOL=${CMAKE_SKIP_BUILD_RPATH}
	-DCMAKE_BUILD_WITH_INSTALL_RPATH:BOOL=${CMAKE_BUILD_WITH_INSTALL_RPATH}
	-DCMAKE_INSTALL_RPATH:PATH=${CMAKE_INSTALL_RPATH}
	-DCMAKE_INSTALL_RPATH_USE_LINK_PATH:BOOL=${CMAKE_INSTALL_RPATH_USE_LINK_PATH}
	-DCMAKE_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
	-DCMAKE_CXX_FLAGS:STRING=${EP_COMMON_CXX_FLAGS}
	-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
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
	GIT_REPOSITORY		${${proj}_REPOSITORY}
	GIT_TAG 			${${proj}_GIT_TAG}
	UPDATE_COMMAND		""	# we are skipping update so we don't have to build every time
	SOURCE_DIR			${${proj}_SOURCE_DIR}
	BINARY_DIR			${${proj}_BUILD_DIR}
	INSTALL_DIR			${${proj}_INSTALL_DIR}
	LOG_DOWNLOAD		${EP_LOG_DOWNLOAD}
	LOG_UPDATE			${EP_LOG_UPDATE}
	LOG_CONFIGURE		${EP_LOG_CONFIGURE}
	LOG_BUILD			${EP_LOG_BUILD}
	LOG_TEST			${EP_LOG_TEST}
	LOG_INSTALL			${EP_LOG_INSTALL}
	CMAKE_GENERATOR		${gen}
	CMAKE_ARGS			${EP_CMAKE_ARGS}
	CMAKE_CACHE_ARGS	${${proj}_CMAKE_OPTIONS}
	DEPENDS				${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( FLANN_DIR ${${proj}_INSTALL_DIR} )
SET( FLANN_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( FLANN_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( FLANN_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )

mark_as_superbuild(
	VARS
		FLANN_DIR:PATH
		FLANN_BUILD_DIR:PATH
		FLANN_INCLUDE_DIR:PATH
		FLANN_LIBRARY_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "FLANN_DIR: ${FLANN_DIR}" )
ExternalProject_Message( ${proj} "FLANN_BUILD_DIR: ${FLANN_BUILD_DIR}" )
ExternalProject_Message( ${proj} "FLANN_INCLUDE_DIR: ${FLANN_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "FLANN_LIBRARY_DIR: ${FLANN_LIBRARY_DIR}" )
### --- End binary information
