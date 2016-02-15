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
	zlib
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

### --- Project specific additions here
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_CONFIGURE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_Boost_configureboost.cmake )
SET( ${proj}_BUILD_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_Boost_buildboost.cmake )

IF( CMAKE_COMPILER_IS_CLANGXX )
	SET( CLANG_ARG -DCMAKE_COMPILER_IS_CLANGXX:BOOL=ON )
ENDIF()

# Boost Git has too many repos and is too slow, so we download instead
SET( BOOST_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )
SET( ${proj}_URL http://sourceforge.net/projects/boost/files/boost/1.60.0/boost_1_60_0.tar.gz )
SET( ${proj}_MD5 28f58b9a33469388302110562bdf6188 )
# SET( ${proj}_REPOSITORY "${git_protocol}://github.com/boostorg/boost.git" )
# SET( ${proj}_GIT_TAG "boost-1.60.0" ) # Dec 23, 2015
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL ${${proj}_URL}
	URL_MD5 ${${proj}_MD5}
	# GIT_REPOSITORY ${${proj}_REPOSITORY}
	# GIT_TAG ${${proj}_GIT_TAG}
	SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj}
	BUILD_IN_SOURCE 1

	${cmakeversion_external_update} "${cmakeversion_external_update_value}"
	CONFIGURE_COMMAND ${CMAKE_COMMAND}
						${CLANG_ARG}
						-DBUILD_DIR:PATH=${CMAKE_CURRENT_BINARY_DIR}/${proj}
						-DCMAKE_C_COMPILER_ID:STRING=${CMAKE_C_COMPILER_ID}
						-DCMAKE_CXX_COMPILER_ID:STRING=${CMAKE_CXX_COMPILER_ID}
						-P ${${proj}_CONFIGURE_SCRIPT}
	
	BUILD_COMMAND ${CMAKE_COMMAND}
						-DBUILD_DIR:PATH=${CMAKE_CURRENT_BINARY_DIR}/${proj}
						-DBOOST_INSTALL_DIR:PATH=${${proj}_INSTALL_DIR}
						-DCMAKE_C_COMPILER_ID:STRING=${CMAKE_C_COMPILER_ID}
						-DCMAKE_CXX_COMPILER_ID:STRING=${CMAKE_CXX_COMPILER_ID}
						-P ${${proj}_BUILD_SCRIPT}
	
	INSTALL_COMMAND ""
	
	DEPENDS
		${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( BOOST_ROOT ${CMAKE_BINARY_DIR}/${proj} )
SET( BOOST_DIR ${CMAKE_BINARY_DIR}/${proj} )
SET( BOOST_LIBRARY_DIR ${CMAKE_BINARY_DIR}/${proj}/lib )
SET( BOOST_INCLUDE_DIR ${CMAKE_BINARY_DIR}/${proj}/boost )

mark_as_superbuild(
	VARS
		BOOST_ROOT:PATH
		BOOST_DIR:PATH
		BOOST_LIBRARY_DIR:PATH
		BOOST_INCLUDE_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "BOOST_ROOT: ${BOOST_ROOT}" )
ExternalProject_Message( ${proj} "BOOST_DIR: ${BOOST_DIR}" )
ExternalProject_Message( ${proj} "BOOST_LIBRARY_DIR: ${BOOST_LIBRARY_DIR}" )
ExternalProject_Message( ${proj} "BOOST_INCLUDE_DIR: ${BOOST_INCLUDE_DIR}" )
### --- End binary information
