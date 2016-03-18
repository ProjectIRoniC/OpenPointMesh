# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName TIFF ) # The find_package known name
SET( proj        TIFF ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	ZLIB
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj} )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( ${proj}_CONFIGURE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_TIFF_configuretiff.cmake )
SET( ${proj}_CONFIGURE_COMMAND
	# CMake ARGS
	${CMAKE_COMMAND}
	-DTIFF_C_FLAGS:STRING=${NONCMAKE_EP_COMMON_C_FLAGS}
	-DTIFF_CXX_FLAGS:STRING=${NONCMAKE_EP_COMMON_CXX_FLAGS}
	-DTIFF_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
	-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
	-DINSTALL_DIR:PATH=${${proj}_INSTALL_DIR}
	-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
	-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
	# ZLIB ARGS
	-DZLIB_INCLUDE_DIR:PATH=${ZLIB_INCLUDE_DIR}
	-DZLIB_LIBRARY_DIR:PATH=${ZLIB_LIBRARY_DIR}
	# Use the configure script
	-P ${${proj}_CONFIGURE_SCRIPT}
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/LuaDist/libtiff/archive/3.8.2.tar.gz )
SET( ${proj}_MD5 c1d8ad4ee235bdef497a23a7d3f51a90 )
# SET( ${proj}_REPOSITORY "${git_protocol}://github.com/LuaDist/libtiff" )
# SET( ${proj}_GIT_TAG "3.8.2" )
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
SET( TIFF_DIR ${${proj}_INSTALL_DIR} )
SET( TIFF_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( TIFF_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( TIFF_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )

mark_as_superbuild(
	VARS
		TIFF_DIR:PATH
		TIFF_BUILD_DIR:PATH
		TIFF_INCLUDE_DIR:PATH
		TIFF_LIBRARY_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "TIFF_DIR: ${TIFF_DIR}" )
ExternalProject_Message( ${proj} "TIFF_BUILD_DIR: ${TIFF_BUILD_DIR}" )
ExternalProject_Message( ${proj} "TIFF_INCLUDE_DIR: ${TIFF_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "TIFF_LIBRARY_DIR: ${TIFF_LIBRARY_DIR}" )
### --- End binary information

# tiff config relies on config.guess, since some dependent files are out of date we are going
# to update them to the latest version
ExternalProject_Add_Step( ${proj} "update config.guess"
	COMMAND ${CMAKE_COMMAND}
		-DUPDATE_GUESS_IN_DIR:PATH=${${proj}_SOURCE_DIR}/config
		-P ${UPDATE_CONFIG_GUESS_SCRIPT}
	
	DEPENDEES download
	DEPENDERS configure
)