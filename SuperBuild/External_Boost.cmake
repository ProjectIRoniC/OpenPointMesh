# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName Boost ) # The find_package known name
SET( proj        Boost ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	BZip2
	ZLIB
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-build )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_LIBRARY_INSTALL_DIR ${${proj}_INSTALL_DIR}/lib )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( ${proj}_CONFIGURE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_Boost_configureboost.cmake )
SET( ${proj}_CONFIGURE_COMMAND
	${CMAKE_COMMAND}
		# CMake Build ARGS
		-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
		-DCMAKE_C_COMPILER_ID:STRING=${CMAKE_C_COMPILER_ID}
		-DCMAKE_CXX_COMPILER_ID:STRING=${CMAKE_CXX_COMPILER_ID}
		# Use the configure script
		-P ${${proj}_CONFIGURE_SCRIPT}
)

SET( ${proj}_BUILD_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_Boost_buildboost.cmake )
SET( ${proj}_BUILD_COMMAND
	${CMAKE_COMMAND}
		# CMake Build ARGS
		-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
		-DBUILD_DIR:PATH=${${proj}_BUILD_DIR}
		-DCMAKE_C_COMPILER_ID:STRING=${CMAKE_C_COMPILER_ID}
		-DCMAKE_CXX_COMPILER_ID:STRING=${CMAKE_CXX_COMPILER_ID}
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
		-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
		-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
		-DEP_NONCMAKE_COMMON_CXX_FLAGS:STRING=${EP_NONCMAKE_COMMON_CXX_FLAGS}
		-DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
		# Boost ARGS
		-DBOOST_INSTALL_DIR:PATH=${${proj}_INSTALL_DIR}
		# bzip2 ARGS
		-DBZIP2_INCLUDE_DIR:PATH=${BZIP2_INCLUDE_DIR}
		-DBZIP2_LIBRARY_DIR:PATH=${BZIP2_LIBRARY_DIR}
		# Python ARGS
		-DPYTHON_INCLUDE_DIRS:PATH=${PYTHON_INCLUDE_DIRS}
		# ZLIB ARGS
		-DZLIB_INCLUDE_DIR:PATH=${ZLIB_INCLUDE_DIR}
		-DZLIB_LIBRARY_DIR:PATH=${ZLIB_LIBRARY_DIR}
		# Use the build script
		-P ${${proj}_BUILD_SCRIPT}
)

# Boost Git has too many repos and is too slow, so we download instead
SET( ${proj}_URL http://sourceforge.net/projects/boost/files/boost/1.60.0/boost_1_60_0.tar.gz )
SET( ${proj}_MD5 28f58b9a33469388302110562bdf6188 )
# SET( ${proj}_REPOSITORY "${git_protocol}://github.com/boostorg/boost.git" )
# SET( ${proj}_GIT_TAG "boost-1.60.0" ) # Dec 23, 2015
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL 				${${proj}_URL}
	URL_MD5				${${proj}_MD5}
	# GIT_REPOSITORY	${${proj}_REPOSITORY}
	# GIT_TAG			${${proj}_GIT_TAG}
	SOURCE_DIR			${${proj}_SOURCE_DIR}
	BINARY_DIR			${${proj}_BUILD_DIR}
	INSTALL_DIR			${${proj}_INSTALL_DIR}
	LOG_DOWNLOAD		${EP_LOG_DOWNLOAD}
	LOG_UPDATE			${EP_LOG_UPDATE}
	LOG_CONFIGURE		${EP_LOG_CONFIGURE}
	LOG_BUILD			${EP_LOG_BUILD}
	LOG_TEST			${EP_LOG_TEST}
	LOG_INSTALL			${EP_LOG_INSTALL}
	CONFIGURE_COMMAND	${${proj}_CONFIGURE_COMMAND}
	BUILD_COMMAND		${${proj}_BUILD_COMMAND}
	INSTALL_COMMAND		"" # Boost b2 already installs, so we skip reapeating the install
	DEPENDS				${${proj}_DEPENDENCIES}
)

IF( BUILD_SHARED_LIBS )
	# this custom step is to copy the Boost library folder to the main project runtime directory
	ExternalProject_Add_Step( ${proj} copy_boost_libraries_to_main_project_runtime_directory
		COMMAND ${CMAKE_COMMAND}
				-E copy_directory ${${proj}_LIBRARY_INSTALL_DIR} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}

			DEPENDEES install
	)
ENDIF()

### --- Set binary information
SET( BOOST_DIR ${${proj}_INSTALL_DIR} )
SET( BOOST_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( BOOST_LIBRARY_DIR ${${proj}_LIBRARY_INSTALL_DIR} )
SET( BOOST_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include/boost-1_60 )

mark_as_superbuild(
	VARS
		BOOST_DIR:PATH
		BOOST_BUILD_DIR:PATH
		BOOST_LIBRARY_DIR:PATH
		BOOST_INCLUDE_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "BOOST_DIR: ${BOOST_DIR}" )
ExternalProject_Message( ${proj} "BOOST_BUILD_DIR: ${BOOST_BUILD_DIR}" )
ExternalProject_Message( ${proj} "BOOST_LIBRARY_DIR: ${BOOST_LIBRARY_DIR}" )
ExternalProject_Message( ${proj} "BOOST_INCLUDE_DIR: ${BOOST_INCLUDE_DIR}" )
### --- End binary information
