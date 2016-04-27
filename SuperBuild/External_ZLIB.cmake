# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName ZLIB ) # The find_package known name
SET( proj        ZLIB ) # The local name
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
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/madler/zlib/archive/v1.2.8.tar.gz )
SET( ${proj}_MD5 1eabf2698dc49f925ce0ffb81397098f )
# SET( ${proj}_REPOSITORY "${git_protocol}://github.com/madler/zlib.git" )
# SET( ${proj}_GIT_TAG "v1.2.8" )
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

# zlib names the library file incorrectly on Windows for what dependents (Qt) expect
# this custom step is to copy the library to the correct name after the install step
IF( WIN32 )
	ExternalProject_Add_Step( ${proj} copy_libraries_in_zlib_folder
		COMMAND ${CMAKE_COMMAND}
				-E copy ${${proj}_INSTALL_DIR}/lib/libzlibstatic.a ${${proj}_INSTALL_DIR}/lib/libz.a

		DEPENDEES install
	)
	
	# Also copy the ones in the main project folder
	ExternalProject_Add_Step( ${proj} copy_libraries_in_primary_project_folder
		COMMAND ${CMAKE_COMMAND}
				-E copy ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libzlibstatic.a ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libz.a

		DEPENDEES install
	)
ENDIF()

### --- Set binary information
SET( ZLIB_DIR ${${proj}_INSTALL_DIR} )
SET( ZLIB_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( ZLIB_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( ZLIB_LIBRARY_DIR ${${proj}_LIBRARY_INSTALL_DIR} )
SET( ZLIB_LIBRARY_NAME z )
	
mark_as_superbuild(
	VARS
		ZLIB_DIR:PATH
		ZLIB_BUILD_DIR:PATH
		ZLIB_INCLUDE_DIR:PATH
		ZLIB_LIBRARY_DIR:PATH
		ZLIB_LIBRARY_NAME:STRING
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "ZLIB_DIR: ${ZLIB_DIR}" )
ExternalProject_Message( ${proj} "ZLIB_BUILD_DIR: ${ZLIB_BUILD_DIR}" )
ExternalProject_Message( ${proj} "ZLIB_INCLUDE_DIR: ${ZLIB_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "ZLIB_LIBRARY_DIR: ${ZLIB_LIBRARY_DIR}" )
### --- End binary information
