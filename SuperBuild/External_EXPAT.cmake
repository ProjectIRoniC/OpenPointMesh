# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName EXPAT ) # The find_package known name
SET( proj        EXPAT ) # The local name
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
SET( ${proj}_LIBRARY_INSTALL_DIR ${${proj}_INSTALL_DIR}/lib )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( ${proj}_CMAKE_OPTIONS
	# CMake Build ARGS
	-DCMAKE_MODULE_PATH:PATH=${CMAKE_MODULE_PATH}
	-DCMAKE_C_STANDARD:STRING=${CMAKE_C_STANDARD}
	-DCMAKE_C_STANDARD_REQUIRED:BOOL=${CMAKE_C_STANDARD_REQUIRED}
	-DCMAKE_INCLUDE_DIRECTORIES_BEFORE:BOOL=${CMAKE_INCLUDE_DIRECTORIES_BEFORE}
	-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
	-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
	-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
	-DCMAKE_MACOSX_RPATH:BOOL=${CMAKE_MACOSX_RPATH}
	-DINSTALL_NAME_DIR:PATH=${${proj}_LIBRARY_INSTALL_DIR}
	-DCMAKE_SKIP_BUILD_RPATH:BOOL=${CMAKE_SKIP_BUILD_RPATH}
	-DCMAKE_BUILD_WITH_INSTALL_RPATH:BOOL=${CMAKE_BUILD_WITH_INSTALL_RPATH}
	-DCMAKE_INSTALL_RPATH:PATH=${${proj}_LIBRARY_INSTALL_DIR}
	-DCMAKE_INSTALL_RPATH_USE_LINK_PATH:BOOL=${CMAKE_INSTALL_RPATH_USE_LINK_PATH}
	-DCMAKE_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
	-DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_C_FLAGS}
	-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
	# EXPAT ARGS
	-DBUILD_shared:BOOL=${BUILD_SHARED_LIBS}
	-DBUILD_examples:BOOL=OFF
	-DBUILD_tests:BOOL=OFF
	-DBUILD_tools:BOOL=ON
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://sourceforge.net/projects/expat/files/expat/2.1.1/expat-2.1.1.tar.bz2/download )
SET( ${proj}_MD5 7380a64a8e3a9d66a9887b01d0d7ea81 )
# SET( ${proj}_REPOSITORY "${git_protocol}://git.code.sf.net/p/expat/code_git expat-code_git.git" )
# SET( ${proj}_GIT_TAG "R_2_1_1" ) # 2.1.1
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
SET( EXPAT_DIR ${${proj}_INSTALL_DIR} )
SET( EXPAT_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( EXPAT_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( EXPAT_LIBRARY_DIR ${${proj}_LIBRARY_INSTALL_DIR} )
SET( EXPAT_LIBRARY_NAME expat )
	
mark_as_superbuild(
	VARS
		EXPAT_DIR:PATH
		EXPAT_BUILD_DIR:PATH
		EXPAT_INCLUDE_DIR:PATH
		EXPAT_LIBRARY_DIR:PATH
		EXPAT_LIBRARY_NAME:STRING
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "EXPAT_DIR: ${EXPAT_DIR}" )
ExternalProject_Message( ${proj} "EXPAT_BUILD_DIR: ${EXPAT_BUILD_DIR}" )
ExternalProject_Message( ${proj} "EXPAT_INCLUDE_DIR: ${EXPAT_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "EXPAT_LIBRARY_DIR: ${EXPAT_LIBRARY_DIR}" )
### --- End binary information
