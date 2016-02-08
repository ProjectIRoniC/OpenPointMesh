# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced
SET( extProjName Boost ) #The find_package known name
SET( proj        Boost ) #This local name
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

IF( NOT ( DEFINED "USE_SYSTEM_${extProjName}" AND "${USE_SYSTEM_${extProjName}}" ) )
	### --- Project specific additions here
	SET( Boost_Install_Dir ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
	SET( Boost_Configure_Script ${CMAKE_CURRENT_LIST_DIR}/External_Boost_configureboost.cmake )
	SET( Boost_Build_Script ${CMAKE_CURRENT_LIST_DIR}/External_Boost_buildboost.cmake )

	# SVN is too slow SVN_REPOSITORY http://svn.boost.org/svn/boost/trunk
	# SVN is too slow SVN_REVISION -r "82586"
	

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
		# GIT_REPOSITORY ${${proj}_REPOSITORY}
		# GIT_TAG ${${proj}_GIT_TAG}
		URL ${${proj}_URL}
		URL_MD5 ${${proj}_MD5}
		SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj}
		BUILD_IN_SOURCE 1

		${cmakeversion_external_update} "${cmakeversion_external_update_value}"
		CONFIGURE_COMMAND ${CMAKE_COMMAND}
							${CLANG_ARG}
							-DBUILD_DIR:PATH=${CMAKE_CURRENT_BINARY_DIR}/${proj}
							-DBOOST_INSTALL_DIR:PATH=${Boost_Install_Dir}
							-DCMAKE_C_COMPILER_ID:STRING=${CMAKE_C_COMPILER_ID}
							-DCMAKE_CXX_COMPILER_ID:STRING=${CMAKE_CXX_COMPILER_ID}
							-P ${Boost_Configure_Script}
		
		BUILD_COMMAND ${CMAKE_COMMAND}
							-DBUILD_DIR:PATH=${CMAKE_CURRENT_BINARY_DIR}/Boost
							-DBOOST_INSTALL_DIR:PATH=${Boost_Install_Dir}
							-DCMAKE_C_COMPILER_ID:STRING=${CMAKE_C_COMPILER_ID}
							-DCMAKE_CXX_COMPILER_ID:STRING=${CMAKE_CXX_COMPILER_ID}
							-P ${Boost_Build_Script}
		
		INSTALL_COMMAND ""
		DEPENDS
			${${proj}_DEPENDENCIES}
	)
	
	### --- Set binary information
	SET( ${extProjName}_ROOT ${CMAKE_BINARY_DIR}/${proj} )
	SET( ${extProjName}_DIR ${CMAKE_BINARY_DIR}/${proj} )
	SET( ${extProjName}_INCLUDE_DIR ${CMAKE_BINARY_DIR}/${proj}/boost )
	### --- End binary information
ELSE()
	IF( ${USE_SYSTEM_${extProjName}} )
		FIND_PACKAGE( ${extProjName} ${${extProjName}_REQUIRED_VERSION} REQUIRED )
		MESSAGE( STATUS "USING the system ${extProjName}, set ${extProjName}_DIR=${${extProjName}_DIR}" )
	ENDIF()
	# The project is provided using ${extProjName}_DIR, nevertheless since other
	# project may depend on ${extProjName}, let's add an 'empty' one
	ExternalProject_Add_Empty( ${proj} "${${proj}_DEPENDENCIES}" )
ENDIF()

mark_as_superbuild(
	VARS
		${extProjName}_ROOT:PATH
		${extProjName}_DIR:PATH
		${extProjName}_INCLUDE_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "Boost_ROOT:${${extProjName}_ROOT}" )
ExternalProject_Message( ${proj} "Boost_DIR:${${extProjName}_DIR}" )
ExternalProject_Message( ${proj} "Boost_INCLUDE_DIR:${${extProjName}_INCLUDE_DIR}" )
