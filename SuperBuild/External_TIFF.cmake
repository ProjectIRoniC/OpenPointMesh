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

### --- Project specific additions here
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_CMAKE_OPTIONS
	-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
	-DCMAKE_CXX_FLAGS:STRING=${ep_common_cxx_flags}
	-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
	-DCMAKE_C_FLAGS:STRING=${ep_common_c_flags}
	-DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
	-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
	-DBUILD_SHARED_LIBS:BOOL=OFF
	-DZLIB_INCLUDE_DIR:PATH=${ZLIB_INCLUDE_DIR}
	-DZLIB_LIBRARY:PATH=${ZLIB_LIBRARY}
	-DJPEG:BOOL=ON
	-DJPEG_LIBRARY:PATH=${JPEG_LIBRARY}
	-DJPEG_INCLUDE_DIR:PATH=${JPEG_INCLUDE_DIR}
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/LuaDist/libtiff/archive/3.8.2.tar.gz )
SET( ${proj}_MD5 c1d8ad4ee235bdef497a23a7d3f51a90 )
# SET( ${proj}_REPOSITORY "${git_protocol}://github.com/LuaDist/libtiff" )
# SET( ${proj}_GIT_TAG "3.8.2" )
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL ${${proj}_URL}
	URL_MD5 ${${proj}_MD5}
	# GIT_REPOSITORY ${${proj}_REPOSITORY}
	# GIT_TAG ${${proj}_GIT_TAG}
	SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj}
	BINARY_DIR ${proj}-build
	LOG_CONFIGURE 0  # Wrap configure in script to ignore log output from dashboards
	LOG_BUILD     0  # Wrap build in script to to ignore log output from dashboards
	LOG_TEST      0  # Wrap test in script to to ignore log output from dashboards
	LOG_INSTALL   0  # Wrap install in script to to ignore log output from dashboards
	${cmakeversion_external_update} "${cmakeversion_external_update_value}"
	INSTALL_DIR ${${proj}_INSTALL_DIR}
	CMAKE_GENERATOR ${gen}
	CMAKE_ARGS -Wno-dev --no-warn-unused-cli
	CMAKE_CACHE_ARGS ${${proj}_CMAKE_OPTIONS}

	DEPENDS
		${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( TIFF_DIR ${CMAKE_BINARY_DIR}/${proj}-install )
SET( TIFF_INCLUDE_DIR ${CMAKE_BINARY_DIR}/${proj}-install/include )
SET( TIFF_LIBRARY_DIR ${CMAKE_BINARY_DIR}/${proj}-install/lib )
SET( TIFF_LIBRARY tiff )
SET( TIFFXX_LIBRARY tiffxx )

mark_as_superbuild(
	VARS
		TIFF_DIR:PATH
		TIFF_INCLUDE_DIR:PATH
		TIFF_LIBRARY_DIR:PATH
		TIFF_LIBRARY:FILEPATH
		TIFFXX_LIBRARY:FILEPATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "TIFF_DIR: ${TIFF_DIR}" )
ExternalProject_Message( ${proj} "TIFF_INCLUDE_DIR: ${TIFF_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "TIFF_LIBRARY_DIR: ${TIFF_LIBRARY_DIR}" )
ExternalProject_Message( ${proj} "TIFF_LIBRARY: ${TIFF_LIBRARY}" )
ExternalProject_Message( ${proj} "TIFFXX_LIBRARY: ${TIFFXX_LIBRARY}" )
### --- End binary information
