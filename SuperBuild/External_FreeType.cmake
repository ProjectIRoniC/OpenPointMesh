# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName Freetype ) # The find_package known name
SET( proj        Freetype ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	BZip2
	PNG
	ZLIB
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-build )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( FREETYPE_CMAKE_PREFIX_PATH
	${BZIP2_DIR}
	${PNG_DIR}
	${ZLIB_DIR}
)

SET( ${proj}_CMAKE_OPTIONS
	# CMake ARGS
	-DCMAKE_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
	-DCMAKE_CXX_FLAGS:STRING=${EP_COMMON_CXX_FLAGS}
	-DCMAKE_PREFIX_PATH:PATH=${FREETYPE_CMAKE_PREFIX_PATH}
	-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
	# BZIP2 ARGS
	-DFT_CONFIG_OPTION_USE_BZIP2:BOOL=ON
	# PNG ARGS
	-DFT_CONFIG_OPTION_USE_PNG:BOOL=ON
	# ZLIB ARGS
	-DFT_CONFIG_OPTION_USE_ZLIB:BOOL=ON
	-DFT_CONFIG_OPTION_SYSTEM_ZLIB:BOOL=ON
	
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL http://git.savannah.gnu.org/cgit/freetype/freetype2.git/snapshot/VER-2-6-3.tar.gz )
SET( ${proj}_MD5 f2d699ed3fa09d12802c9b032510b54d )
# SET( ${proj}_REPOSITORY "${git_protocol}://git.savannah.gnu.org/cgit/freetype/freetype2.git" )
# SET( ${proj}_GIT_TAG "VER-2-6-3" )
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL					${${proj}_URL}
	#URL_MD5			${${proj}_MD5}	 # For some reason the md5 changes on this webiste
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
SET( FREETYPE_DIR ${${proj}_INSTALL_DIR} )
SET( FREETYPE_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( FREETYPE_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include/freetype2 )
SET( FREETYPE_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )
SET( FREETYPE_LIBRARY_NAME freetype )
	
mark_as_superbuild(
	VARS
		FREETYPE_DIR:PATH
		FREETYPE_BUILD_DIR:PATH
		FREETYPE_INCLUDE_DIR:PATH
		FREETYPE_LIBRARY_DIR:PATH
		FREETYPE_LIBRARY_NAME:STRING
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "FREETYPE_DIR: ${FREETYPE_DIR}" )
ExternalProject_Message( ${proj} "FREETYPE_BUILD_DIR: ${FREETYPE_BUILD_DIR}" )
ExternalProject_Message( ${proj} "FREETYPE_INCLUDE_DIR: ${FREETYPE_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "FREETYPE_LIBRARY_DIR: ${FREETYPE_LIBRARY_DIR}" )
ExternalProject_Message( ${proj} "FREETYPE_LIBRARY_NAME: ${FREETYPE_LIBRARY_NAME}" )
### --- End binary information
