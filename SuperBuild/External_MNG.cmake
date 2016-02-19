# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName MNG ) # The find_package known name
SET( proj        MNG ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	JPEG
	zlib
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

### --- Project specific additions here
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_CONFIGURE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_MNG_configuremng.cmake )

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/LuaDist/libmng/archive/1.0.10.tar.gz )
SET( ${proj}_MD5 e63cbc9ce44f12663e269b2268bce3bb )
# SET( ${proj}_REPOSITORY "${git_protocol}://github.com/LuaDist/libmng" )
# SET( ${proj}_GIT_TAG "1.0.10" )
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
						-DBUILD_DIR:PATH=${CMAKE_CURRENT_BINARY_DIR}/${proj}
						-DINSTALL_DIR:PATH=${${proj}_INSTALL_DIR}
						-DCMAKE_C_COMPILER_ID:STRING=${CMAKE_C_COMPILER_ID}
						-DCMAKE_CXX_COMPILER_ID:STRING=${CMAKE_CXX_COMPILER_ID}
						-DZLIB_DIR:PATH=${ZLIB_DIR}
						-DJPEG_DIR:PATH=${JPEG_DIR}
						-P ${${proj}_CONFIGURE_SCRIPT}
	
	BUILD_COMMAND make
	
	INSTALL_COMMAND make install

	DEPENDS
		${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( MNG_DIR ${CMAKE_BINARY_DIR}/${proj}-install )
SET( MNG_INCLUDE_DIR ${CMAKE_BINARY_DIR}/${proj}-install/include )
SET( MNG_LIBRARY_DIR ${CMAKE_BINARY_DIR}/${proj}-install/lib )
SET( MNG_LIBRARY mng )

mark_as_superbuild(
	VARS
		MNG_DIR:PATH
		MNG_INCLUDE_DIR:PATH
		MNG_LIBRARY_DIR:PATH
		MNG_LIBRARY:FILEPATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "MNG_DIR: ${MNG_DIR}" )
ExternalProject_Message( ${proj} "MNG_INCLUDE_DIR: ${MNG_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "MNG_LIBRARY_DIR: ${MNG_LIBRARY_DIR}" )
ExternalProject_Message( ${proj} "MNG_LIBRARY: ${MNG_LIBRARY}" )
### --- End binary information

# MNG autogen.sh is no longer maintained so it is in a folder called unmaintained
# this custom step is to move autogen.sh to the build directory
SET( ${proj}_MOVE_AUTOGEN ${CMAKE_CURRENT_LIST_DIR}/External_MNG_moveautogen.cmake )
	
ExternalProject_Add_Step( ${proj} "moveautogen"
	COMMAND ${CMAKE_COMMAND}
		-DMNG_BUILD_DIR:PATH=${SOURCE_DOWNLOAD_CACHE}/${proj}
		-DMNG_AUTOGEN_DIR:PATH=${SOURCE_DOWNLOAD_CACHE}/${proj}/unmaintained
		-P ${${proj}_MOVE_AUTOGEN}
	
	DEPENDEES
		download
)
