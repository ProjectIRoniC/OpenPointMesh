# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName PNG ) # The find_package known name
SET( proj        PNG ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	zlib
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-build )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( PNG_CMAKE_PREFIX_PATH
	${ZLIB_DIR}
)

SET( ${proj}_CMAKE_OPTIONS
	# CMake Build ARGS
	-DCMAKE_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
	-DCMAKE_PREFIX_PATH:PATH=${PNG_CMAKE_PREFIX_PATH}
	-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
	# PNG ARGS
	-DPNG_SHARED:BOOL=OFF
	-DPNG_TESTS:BOOL=OFF
	# ZLIB ARGS
	-DZLIB_LIBRARY:FILEPATH=${ZLIB_LIBRARY}
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/LuaDist/libpng/archive/1.5.10.tar.gz )
SET( ${proj}_MD5 47f5f8c0488f41f4e71d1c9e922c79d4 )
# SET( ${proj}_REPOSITORY "${git_protocol}://github.com/LuaDist/libpng" )
# SET( ${proj}_GIT_TAG "1.5.10" )
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
SET( PNG_DIR ${${proj}_INSTALL_DIR} )
SET( PNG_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( PNG_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( PNG_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )

mark_as_superbuild(
	VARS
		PNG_DIR:PATH
		PNG_BUILD_DIR:PATH
		PNG_INCLUDE_DIR:PATH
		PNG_LIBRARY_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "PNG_DIR: ${PNG_DIR}" )
ExternalProject_Message( ${proj} "PNG_BUILD_DIR: ${PNG_BUILD_DIR}" )
ExternalProject_Message( ${proj} "PNG_INCLUDE_DIR: ${PNG_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "PNG_LIBRARY_DIR: ${PNG_LIBRARY_DIR}" )
### --- End binary information
