# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName TIFF ) # The find_package known name
SET( proj        TIFF ) # The local name
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

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-build )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( ${proj}_CONFIGURE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_TIFF_configuretiff.cmake )
SET( ${proj}_CONFIGURE_COMMAND
	${CMAKE_COMMAND}
	# CMake Build ARGS
	-DTIFF_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
	-DTIFF_CXX_FLAGS:STRING=${EP_COMMON_CXX_FLAGS}
	-DBUILD_DIR:PATH=${CMAKE_CURRENT_BINARY_DIR}/${proj}
	-DINSTALL_DIR:PATH=${${proj}_INSTALL_DIR}
	-DCMAKE_C_COMPILER_ID:STRING=${CMAKE_C_COMPILER_ID}
	-DCMAKE_CXX_COMPILER_ID:STRING=${CMAKE_CXX_COMPILER_ID}
	# ZLIB ARGS
	-DZLIB_INCLUDE_DIR:PATH=${ZLIB_INCLUDE_DIR}
	-DZLIB_LIBRARY_DIR:PATH=${ZLIB_LIBRARY_DIR}
	# JPEG ARGS
	-DJPEG_INCLUDE_DIR:PATH=${JPEG_INCLUDE_DIR}
	-DJPEG_LIBRARY_DIR:PATH=${JPEG_LIBRARY_DIR}
	# Use the configure script
	-P ${${proj}_CONFIGURE_SCRIPT}
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/LuaDist/libtiff/archive/3.8.2.tar.gz )
SET( ${proj}_MD5 c1d8ad4ee235bdef497a23a7d3f51a90 )
# SET( ${proj}_REPOSITORY "${git_protocol}://github.com/LuaDist/libtiff" )
# SET( ${proj}_GIT_TAG "3.8.2" )
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL		${${proj}_URL}
	URL_MD5	${${proj}_MD5}
	# GIT_REPOSITORY	${${proj}_REPOSITORY}
	# GIT_TAG			${${proj}_GIT_TAG}
	SOURCE_DIR	${${proj}_SOURCE_DIR}
	BUILD_IN_SOURCE 1
	${cmakeversion_external_update} "${cmakeversion_external_update_value}"
	CONFIGURE_COMMAND	${${proj}_CONFIGURE_COMMAND}
	INSTALL_COMMAND		make install
	DEPENDS ${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( TIFF_DIR ${${proj}_INSTALL_DIR} )
SET( TIFF_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( TIFF_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( TIFF_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )
SET( TIFF_LIBRARY tiff )
SET( TIFFXX_LIBRARY tiffxx )

mark_as_superbuild(
	VARS
		TIFF_DIR:PATH
		TIFF_BUILD_DIR:PATH
		TIFF_INCLUDE_DIR:PATH
		TIFF_LIBRARY_DIR:PATH
		TIFF_LIBRARY:FILEPATH
		TIFFXX_LIBRARY:FILEPATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "TIFF_DIR: ${TIFF_DIR}" )
ExternalProject_Message( ${proj} "TIFF_BUILD_DIR: ${TIFF_BUILD_DIR}" )
ExternalProject_Message( ${proj} "TIFF_INCLUDE_DIR: ${TIFF_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "TIFF_LIBRARY_DIR: ${TIFF_LIBRARY_DIR}" )
ExternalProject_Message( ${proj} "TIFF_LIBRARY: ${TIFF_LIBRARY}" )
ExternalProject_Message( ${proj} "TIFFXX_LIBRARY: ${TIFFXX_LIBRARY}" )
### --- End binary information

# tiff config relies on config.guess, since some dependent files are out of date we are going
# to update them to the latest version
SET( ${proj}_UPDATE_CONFIG_GUESS_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_updateguess.cmake )

ExternalProject_Add_Step( ${proj} "update config.guess"
	COMMAND ${CMAKE_COMMAND}
		-DBUILD_DIR:PATH=${CMAKE_CURRENT_BINARY_DIR}/${proj}/config
		-P ${${proj}_UPDATE_CONFIG_GUESS_SCRIPT}
	
	DEPENDEES download
)
