# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName LCMS ) # The find_package known name
SET( proj        LCMS ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	JPEG
	TIFF
	ZLIB
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-build )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( LCMS_CMAKE_PREFIX_PATH
	${JPEG_DIR}
	${TIFF_DIR}
	${ZLIB_DIR}
)

SET( ${proj}_CMAKE_OPTIONS
	# CMake Build ARGS
	-DCMAKE_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
	-DCMAKE_CXX_FLAGS:STRING=${EP_COMMON_CXX_FLAGS}
	-DCMAKE_PREFIX_PATH:PATH=${LCMS_CMAKE_PREFIX_PATH}
	-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/LuaDist/lcms/archive/1.19.tar.gz )
SET( ${proj}_MD5 e4232b8213974761c553a0d92acac504 )
# SET( ${proj}_REPOSITORY "${git_protocol}://github.com/LuaDist/lcms" )
# SET( ${proj}_GIT_TAG "1.19" )
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL					${${proj}_URL}
	URL_MD5				${${proj}_MD5}
	# GIT_REPOSITORY	${${proj}_REPOSITORY}
	# GIT_TAG 			${${proj}_GIT_TAG}
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

# lcms config relies on config.guess, since some dependent files are out of date we are going
# to update them to the latest version
ExternalProject_Add_Step( ${proj} "update config.guess"
	COMMAND ${CMAKE_COMMAND}
		-DUPDATE_GUESS_IN_DIR:PATH=${${proj}_SOURCE_DIR}
		-P ${UPDATE_CONFIG_GUESS_SCRIPT}
	
	DEPENDEES download
	DEPENDERS configure
)

### --- Set binary information
SET( LCMS_DIR ${${proj}_INSTALL_DIR} )
SET( LCMS_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( LCMS_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( LCMS_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )
SET( LCMS_LIBRARY_NAME lcms )

mark_as_superbuild(
	VARS
		LCMS_DIR:PATH
		LCMS_BUILD_DIR:PATH
		LCMS_INCLUDE_DIR:PATH
		LCMS_LIBRARY_DIR:PATH
		LCMS_LIBRARY_NAME:STRING
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "LCMS_DIR: ${LCMS_DIR}" )
ExternalProject_Message( ${proj} "LCMS_BUILD_DIR: ${LCMS_BUILD_DIR}" )
ExternalProject_Message( ${proj} "LCMS_INCLUDE_DIR: ${LCMS_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "LCMS_LIBRARY_DIR: ${LCMS_LIBRARY_DIR}" )
ExternalProject_Message( ${proj} "LCMS_LIBRARY_NAME: ${LCMS_LIBRARY_NAME}" )
### --- End binary information
