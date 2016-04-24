# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName BZip2 ) # The find_package known name
SET( proj        BZip2 ) # The local name
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
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj} )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( ${proj}_BUILD_COMMAND make )

IF( BUILD_SHARED_LIBS AND NOT APPLE )
	SET( ${proj}_BUILD_COMMAND make -f Makefile-libbz2_so )
ENDIF()

SET( ${proj}_INSTALL_COMMAND make install PREFIX=${${proj}_INSTALL_DIR} )

# Download tar source when possible to speed up build time
SET( ${proj}_URL http://www.bzip.org/1.0.6/bzip2-1.0.6.tar.gz )
SET( ${proj}_MD5 00b516f4704d4a7cb50a1d97e6e8e15b )
# SET( ${proj}_REPOSITORY "${git_protocol}://github.com/enthought/bzip2.git" )
# SET( ${proj}_GIT_TAG "v1.0.6" )
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL					${${proj}_URL}
	URL_MD5				${${proj}_MD5}
	# GIT_REPOSITORY	${${proj}_REPOSITORY}
	# GIT_TAG 			${${proj}_GIT_TAG}
	SOURCE_DIR			${${proj}_SOURCE_DIR}
	BUILD_IN_SOURCE		1
	INSTALL_DIR			${${proj}_INSTALL_DIR}
	LOG_DOWNLOAD		${EP_LOG_DOWNLOAD}
	LOG_UPDATE			${EP_LOG_UPDATE}
	LOG_CONFIGURE		${EP_LOG_CONFIGURE}
	LOG_BUILD			${EP_LOG_BUILD}
	LOG_TEST			${EP_LOG_TEST}
	LOG_INSTALL			${EP_LOG_INSTALL}
	CONFIGURE_COMMAND	"" # there is no need to configure bzip2
	BUILD_COMMAND		${${proj}_BUILD_COMMAND}
	INSTALL_COMMAND		${${proj}_INSTALL_COMMAND}
	DEPENDS				${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( BZIP2_DIR ${${proj}_INSTALL_DIR} )
SET( BZIP2_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( BZIP2_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( BZIP2_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )
SET( BZIP2_LIBRARY_NAME bz2 )

mark_as_superbuild(
	VARS
		BZIP2_DIR:PATH
		BZIP2_BUILD_DIR:PATH
		BZIP2_INCLUDE_DIR:PATH
		BZIP2_LIBRARY_DIR:PATH
		BZIP2_LIBRARY_NAME:STRING
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "BZIP2_DIR: ${BZIP2_DIR}" )
ExternalProject_Message( ${proj} "BZIP2_BUILD_DIR: ${BZIP2_BUILD_DIR}" )
ExternalProject_Message( ${proj} "BZIP2_INCLUDE_DIR: ${BZIP2_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "BZIP2_LIBRARY_DIR: ${BZIP2_LIBRARY_DIR}" )
### --- End binary information
