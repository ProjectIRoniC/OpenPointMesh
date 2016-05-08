# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName FontConfig ) # The find_package known name
SET( proj        FontConfig ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	EXPAT
	Freetype
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj} )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_LIBRARY_INSTALL_DIR ${${proj}_INSTALL_DIR}/lib )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )
SET( ${proj}_CACHEDIR ${${proj}_SOURCE_DIR}/fc_cache )

### --- Project specific additions here
SET( FONTCONFIG_CMAKE_PREFIX_PATH
	${BZIP2_DIR}
	${EXPAT_DIR}
	${FREETYPE_DIR}
	${PNG_DIR}
	${ZLIB_DIR}
)

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
	-DCMAKE_PREFIX_PATH:PATH=${FONTCONFIG_CMAKE_PREFIX_PATH}
	-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
)

# Download tar source when possible to speed up build time
# We are using a repository where CMakeLists.txt is being maintained for fontconifg
#SET( ${proj}_URL https://github.com/Mixaill/cmake.fontconfig/archive/2.11.1.tar.gz )
#SET( ${proj}_MD5 6424af76094b271cfad54d76c9919649 )
SET( ${proj}_REPOSITORY "${git_protocol}://github.com/Mixaill/cmake.fontconfig.git" )
SET( ${proj}_GIT_TAG "cmake" )

# We are patching:
#	src/CMakeLists.txt to include fcrange.c
#	conf.d/CMakeLists.txt to install the README.in file instead of README since it is not generated
#	CMakeLists.txt to not overwrite our CMAKE_MODULES_PATH
#	autogen.sh to use glibtoolize instead of libtoolize since GUN libtoolize is named differently on OS X
SET( ${proj}_PATCH_COMMAND patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/External_FontConfig_patchfontconfig.diff )
### --- End Project specfic additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	# URL				${${proj}_URL}
	# URL_MD5			${${proj}_MD5}
	GIT_REPOSITORY		${${proj}_REPOSITORY}
	GIT_TAG 			${${proj}_GIT_TAG}
	UPDATE_COMMAND		""	# we are skipping update so we don't have to build every time
	SOURCE_DIR			${${proj}_SOURCE_DIR}
	BUILD_IN_SOURCE		1
	INSTALL_DIR			${${proj}_INSTALL_DIR}
	LOG_DOWNLOAD		${EP_LOG_DOWNLOAD}
	LOG_UPDATE			${EP_LOG_UPDATE}
	LOG_CONFIGURE		${EP_LOG_CONFIGURE}
	LOG_BUILD			${EP_LOG_BUILD}
	LOG_TEST			${EP_LOG_TEST}
	LOG_INSTALL			${EP_LOG_INSTALL}
	PATCH_COMMAND		${${proj}_PATCH_COMMAND}
	CMAKE_GENERATOR		${gen}
	CMAKE_ARGS			${EP_CMAKE_ARGS}
	CMAKE_CACHE_ARGS	${${proj}_CMAKE_OPTIONS}
	DEPENDS				${${proj}_DEPENDENCIES}
)

# Files: "fcalias.h fcaliastail.h fcftalias.h fcftaliastail.h" generated on Ubuntu by ./configure & make
# We need to run autogen.sh because it creates a needed file fcstdint.h
SET( ${proj}_PREBUILD_CONFIGURE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_FontConfig_preconfigurefontconfig.cmake )
ExternalProject_Add_Step( ${proj} run_autogen_sh
	COMMAND ${CMAKE_COMMAND}
		# CMake ARGS
		-DEP_NONCMAKE_COMMON_C_FLAGS:STRING=${EP_NONCMAKE_COMMON_C_FLAGS}
		-DCMAKE_EXE_LINKER_C_FLAGS:STRING=${CMAKE_EXE_LINKER_C_FLAGS}
		-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
		-DINSTALL_DIR:PATH=${${proj}_INSTALL_DIR}
		-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
		-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
		# Autogen ARGS
		-DRUN_AUTOGENSH_SCRIPT:FILEPATH=${RUN_AUTOGENSH_SCRIPT}
		# EXPAT ARGS
		-DEXPAT_DIR:PATH=${EXPAT_DIR}
		-DEXPAT_INCLUDE_DIR:PATH=${EXPAT_INCLUDE_DIR}
		-DEXPAT_LIBRARY_DIR:PATH=${EXPAT_LIBRARY_DIR}
		# FreeType ARGS
		-DFREETYPE_INCLUDE_DIR:PATH=${FREETYPE_INCLUDE_DIR}
		-DFREETYPE_LIBRARY_DIR:PATH=${FREETYPE_LIBRARY_DIR}
		-DFREETYPE_LIBRARY_NAME:STRING=${FREETYPE_LIBRARY_NAME}
		# bizp2 ARGS
		-DBZIP2_LIBRARY_DIR:PATH=${BZIP2_LIBRARY_DIR}
		-DBZIP2_LIBRARY_NAME:STRING=${BZIP2_LIBRARY_NAME}
		# PNG ARGS
		-DPNG_LIBRARY_DIR:PATH=${PNG_LIBRARY_DIR}
		-DPNG_LIBRARY_NAME:STRING=${PNG_LIBRARY_NAME}
		# ZLIB ARGS
		-DZLIB_LIBRARY_DIR:PATH=${ZLIB_LIBRARY_DIR}
		-DZLIB_LIBRARY_NAME:STRING=${ZLIB_LIBRARY_NAME}
		# Use the configure script
		-P ${${proj}_PREBUILD_CONFIGURE_SCRIPT}

	DEPENDEES patch
	DEPENDERS configure
)

# Generate alias headers
SET( ${proj}_GENERATE_ALIAS_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_FontConfig_generatealias.cmake )
ExternalProject_Add_Step( ${proj} generate_alias
	COMMAND ${CMAKE_COMMAND}
		# CMake ARGS
		-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
		# Use the configure script
		-P ${${proj}_GENERATE_ALIAS_SCRIPT}

	DEPENDEES run_autogen_sh
	DEPENDERS configure
)

### --- Set binary information
SET( FONTCONFIG_DIR ${${proj}_INSTALL_DIR} )
SET( FONTCONFIG_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( FONTCONFIG_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( FONTCONFIG_LIBRARY_DIR ${${proj}_LIBRARY_INSTALL_DIR} )
SET( FONTCONFIG_LIBRARY_NAME fontconfig )
	
mark_as_superbuild(
	VARS
		FONTCONFIG_DIR:PATH
		FONTCONFIG_BUILD_DIR:PATH
		FONTCONFIG_INCLUDE_DIR:PATH
		FONTCONFIG_LIBRARY_DIR:PATH
		FONTCONFIG_LIBRARY_NAME:STRING
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "FONTCONFIG_DIR: ${FONTCONFIG_DIR}" )
ExternalProject_Message( ${proj} "FONTCONFIG_BUILD_DIR: ${FONTCONFIG_BUILD_DIR}" )
ExternalProject_Message( ${proj} "FFONTCONFIG_INCLUDE_DIR: ${FONTCONFIG_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "FONTCONFIG_LIBRARY_DIR: ${FONTCONFIG_LIBRARY_DIR}" )
### --- End binary information
