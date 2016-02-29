# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName OpenNI2 ) # The find_package known name
SET( proj        OpenNI2 ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE(FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})")
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	libusb
	GraphViz
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-build )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( OPENNI2_CMAKE_PREFIX_PATH
	${LIBUSB_DIR}
	${GRAPHVIZ_DIR}
)

SET( ${proj}_CMAKE_OPTIONS
	# CMake Build ARGS
	-DCMAKE_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
	-DCMAKE_CXX_FLAGS:STRING=${EP_COMMON_CXX_FLAGS}
	-DCMAKE_PREFIX_PATH:PATH=${OPENNI2_CMAKE_PREFIX_PATH}
	-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
)

#SET( ${proj}_BUILD_COMMAND
#	msbuild
#	${CMAKE_CURRENT_BINARY_DIR}/${proj}/OpenNI.sln
#	/p:Configuration=Release
#)

#SET( ${proj}_CONFIGURE_COMMAND
#	devenv
#	${CMAKE_CURRENT_BINARY_DIR}/${proj}/OpenNI.sln
#	/upgrade
#)

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/occipital/OpenNI2/archive/v2.2.0-debian.tar.gz )
SET( ${proj}_MD5 bdb95be379150c6bd0433f8a6862ee7f )
#SET( ${proj}_REPOSITORY "${git_protocol}://github.com/occipital/OpenNI2.git" )
#SET( ${proj}_GIT_TAG v2.2.0-debian )  # 2.2.0-debian
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

### --- Set binary information
SET( OPENNI2_DIR ${${proj}_INSTALL_DIR} )
SET( OPENNI2_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( OPENNI2_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( OPENNI2_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )
### --- End binary information

mark_as_superbuild(
  VARS
    OPENNI2_DIR:PATH
	OPENNI2_BUILD_DIR:PATH
	OPENNI2_INCLUDE_DIR:PATH
	OPENNI2_LIBRARY_DIR:PATH
  LABELS
     "FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "OPENNI2_DIR: ${OPENNI2_DIR}" )
ExternalProject_Message( ${proj} "OPENNI2_BUILD_DIR: ${OPENNI2_BUILD_DIR}" )
ExternalProject_Message( ${proj} "OPENNI2_INCLUDE_DIR: ${OPENNI2_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "OPENNI2_LIBRARY_DIR: ${OPENNI2_LIBRARY_DIR}" )
