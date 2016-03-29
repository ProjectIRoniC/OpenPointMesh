# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName GraphViz ) # The find_package known name
SET( proj        GraphViz ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	EXPAT
	FontConfig
	Freetype
	ZLIB
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj} )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( GRAPHVIZ_C_FLAGS "${EP_NONCMAKE_COMMON_C_FLAGS}" )
SET( GRAPHVIZ_CXX_FLAGS "${EP_NONCMAKE_COMMON_CXX_FLAGS}" )

# Hack to fix multiple definitions in static builds
# Link Time Optimization might be able to fix this when we get that working
#IF( NOT BUILD_SHARED_LIBS )
#	SET( GRAPHVIZ_C_FLAGS "${GRAPHVIZ_C_FLAGS} -Wl,--allow-multiple-definition" )
#	SET( GRAPHVIZ_CXX_FLAGS "${GRAPHVIZ_CXX_FLAGS} -Wl,--allow-multiple-definition" )
#ENDIF()

SET( ${proj}_CONFIGURE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_GraphViz_configuregraphviz.cmake )
SET( ${proj}_CONFIGURE_COMMAND
	${CMAKE_COMMAND}
	# CMake Build ARGS
	-DGRAPHVIZ_C_FLAGS:STRING=${GRAPHVIZ_C_FLAGS}
	-DGRAPHVIZ_CXX_FLAGS:STRING=${GRAPHVIZ_CXX_FLAGS}
	-DGRAPHVIZ_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
	-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
	-DINSTALL_DIR:PATH=${${proj}_INSTALL_DIR}
	-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
	-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
	# EXPAT ARGS
	-DEXPAT_INCLUDE_DIR:PATH=${EXPAT_INCLUDE_DIR}
	-DEXPAT_LIBRARY_DIR:PATH=${EXPAT_LIBRARY_DIR}
	# FONTCONFIG ARGS
	-DFONTCONFIG_LIBRARY_DIR:PATH=${FONTCONFIG_LIBRARY_DIR}
	# FREETYPE ARGS
	-DFREETYPE_INCLUDE_DIR:PATH=${FREETYPE_INCLUDE_DIR}
	-DFREETYPE_LIBRARY_DIR:PATH=${FREETYPE_LIBRARY_DIR}
	# ZLIB ARGS
	-DZLIB_INCLUDE_DIR:PATH=${ZLIB_INCLUDE_DIR}
	-DZLIB_LIBRARY_DIR:PATH=${ZLIB_LIBRARY_DIR}
	# Use the configure script
	-P ${${proj}_CONFIGURE_SCRIPT}
)

# Download tar source when possible to speed up build time
# SET( ${proj}_URL http://www.graphviz.org/pub/graphviz/stable/SOURCES/graphviz-2.38.0.tar.gz )
# SET( ${proj}_MD5 5b6a829b2ac94efcd5fa3c223ed6d3ae )
SET( ${proj}_REPOSITORY "${git_protocol}://github.com/ellson/graphviz" )
SET( ${proj}_GIT_TAG "master" )
### --- End Project specific additions

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
	CONFIGURE_COMMAND	${${proj}_CONFIGURE_COMMAND}
	DEPENDS				${${proj}_DEPENDENCIES}
)

# Since we are downloading from master we need to run autogen.sh before configure
ExternalProject_Add_Step( ${proj} "run autogen.sh"
	COMMAND ${CMAKE_COMMAND}
		-DAUTOGENSH_IN_DIR:PATH=${${proj}_SOURCE_DIR}
		-P ${RUN_AUTOGENSH_SCRIPT}
	DEPENDEES download
	DEPENDERS configure
)

### --- Set binary information
SET( GRAPHVIZ_DIR ${${proj}_INSTALL_DIR} )
SET( GRAPHVIZ_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( GRAPHVIZ_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( GRAPHVIZ_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )
	
mark_as_superbuild(
	VARS
		GRAPHVIZ_DIR:PATH
		GRAPHVIZ_BUILD_DIR:PATH
		GRAPHVIZ_INCLUDE_DIR:PATH
		GRAPHVIZ_LIBRARY_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "GRAPHVIZ_DIR: ${GRAPHVIZ_DIR}" )
ExternalProject_Message( ${proj} "GRAPHVIZ_BUILD_DIR: ${GRAPHVIZ_BUILD_DIR}" )
ExternalProject_Message( ${proj} "GRAPHVIZ_INCLUDE_DIR: ${GRAPHVIZ_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "GRAPHVIZ_LIBRARY_DIR: ${GRAPHVIZ_LIBRARY_DIR}" )
### --- End binary information
