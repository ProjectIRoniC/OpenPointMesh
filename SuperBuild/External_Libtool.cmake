# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName Libtool ) # The find_package known name
SET( proj        Libtool ) # The local name
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
SET( ${proj}_CONFIGURE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_Libtool_configurelibtool.cmake )
SET( ${proj}_CONFIGURE_COMMAND
	# CMake ARGS
	${CMAKE_COMMAND}
	-DEP_NONCMAKE_COMMON_C_FLAGS:STRING=${EP_NONCMAKE_COMMON_C_FLAGS}
	-DCMAKE_EXE_LINKER_C_FLAGS:STRING=${CMAKE_EXE_LINKER_C_FLAGS}
	-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
	-DINSTALL_DIR:PATH=${${proj}_INSTALL_DIR}
	-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
	-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
	# Use the configure script
	-P ${${proj}_CONFIGURE_SCRIPT}
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL http://ftpmirror.gnu.org/libtool/libtool-2.2.10.tar.gz )
SET( ${proj}_MD5 b745d220e88163fcd9eea0a90ccf21b0 )
# SET( ${proj}_REPOSITORY "${git_protocol}://git.savannah.gnu.org/libtool.git" )
# SET( ${proj}_GIT_TAG "v2.4.6" )
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL					${${proj}_URL}
	URL_MD5				${${proj}_MD5}
	# GIT_REPOSITORY	${${proj}_REPOSITORY}
	# GIT_TAG			${${proj}_GIT_TAG}
	SOURCE_DIR			${${proj}_SOURCE_DIR}
	BUILD_IN_SOURCE		1
	INSTALL_DIR			${${proj}_INSTALL_DIR}
	LOG_DOWNLOAD		${EP_LOG_DOWNLOAD}
	LOG_UPDATE			${EP_LOG_UPDATE}
	LOG_CONFIGURE		${EP_LOG_CONFIGURE}
	LOG_BUILD			${EP_LOG_BUILD}
	LOG_TEST			${EP_LOG_TEST}
	LOG_INSTALL			${EP_LOG_INSTALL}
	CONFIGURE_COMMAND	${${proj}_CONFIGURE_COMMAND}
	DEPENDS 			${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( LIBTOOL_DIR ${${proj}_INSTALL_DIR} )
SET( LIBTOOL_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( LIBTOOL_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( LIBTOOL_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )
	
mark_as_superbuild(
	VARS
		LIBTOOL_DIR:PATH
		LIBTOOL_BUILD_DIR:PATH
		LIBTOOL_INCLUDE_DIR:PATH
		LIBTOOL_LIBRARY_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "LIBTOOL_DIR: ${LIBTOOL_DIR}" )
ExternalProject_Message( ${proj} "LIBTOOL_BUILD_DIR: ${LIBTOOL_BUILD_DIR}" )
ExternalProject_Message( ${proj} "LIBTOOL_INCLUDE_DIR: ${LIBTOOL_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "LIBTOOL_LIBRARY_DIR: ${LIBTOOL_LIBRARY_DIR}" )
### --- End binary information
