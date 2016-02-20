# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName Qt ) # The find_package known name
SET( proj        Qt ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	zlib
	PNG
	MNG
	JPEG
	TIFF
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

### --- Project specific additions here
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_CONFIGURE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_Qt_configureqt.cmake )

SET( ${proj}_URL https://download.qt.io/archive/qt/4.8/4.8.6/qt-everywhere-opensource-src-4.8.6.tar.gz )
SET( ${proj}_MD5 2edbe4d6c2eff33ef91732602f3518eb )
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL ${${proj}_URL}
	URL_MD5 ${${proj}_MD5}
	SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj}
	BUILD_IN_SOURCE 1
	
	${cmakeversion_external_update} "${cmakeversion_external_update_value}"
	CONFIGURE_COMMAND ${CMAKE_COMMAND}
						-DBUILD_DIR:PATH=${CMAKE_CURRENT_BINARY_DIR}/${proj}
						-DCMAKE_C_COMPILER_ID:STRING=${CMAKE_C_COMPILER_ID}
						-DCMAKE_CXX_COMPILER_ID:STRING=${CMAKE_CXX_COMPILER_ID}
						-DZLIB_INCLUDE_DIR:PATH=${ZLIB_INCLUDE_DIR}
						-DZLIB_LIBRARY_DIR:PATH=${ZLIB_LIBRARY_DIR}
						-DZLIB_LIBRARY:FILEPATH=${ZLIB_LIBRARY}
						-DTIFF_INCLUDE_DIR:PATH=${TIFF_INCLUDE_DIR}
						-DTIFF_LIBRARY_DIR:PATH=${TIFF_LIBRARY_DIR}
						-DTIFF_LIBRARY:FILEPATH=${TIFF_LIBRARY}
						-DTIFFXX_LIBRARY:FILEPATH=${TIFFXX_LIBRARY}
						-DPNG_INCLUDE_DIR:PATH=${PNG_INCLUDE_DIR}
						-DPNG_LIBRARY_DIR:PATH=${PNG_LIBRARY_DIR}
						-DPNG_LIBRARY:FILEPATH=${PNG_LIBRARY}
						-DMNG_INCLUDE_DIR:PATH=${MNG_INCLUDE_DIR}
						-DMNG_LIBRARY_DIR:PATH=${MNG_LIBRARY_DIR}
						-DMNG_LIBRARY:FILEPATH=${MNG_LIBRARY}
						-DJPEG_INCLUDE_DIR:PATH=${JPEG_INCLUDE_DIR}
						-DJPEG_LIBRARY_DIR:PATH=${JPEG_LIBRARY_DIR}
						-DJPEG_LIBRARY:FILEPATH=${JPEG_LIBRARY}
						-DLCMS_INCLUDE_DIR:PATH=${LCMS_INCLUDE_DIR}
						-DLCMS_LIBRARY_DIR:PATH=${LCMS_LIBRARY_DIR}
						-DLCMS_LIBRARY:FILEPATH=${LCMS_LIBRARY}
						-P ${${proj}_CONFIGURE_SCRIPT}
	
	BUILD_COMMAND make

	INSTALL_COMMAND ""
	
	DEPENDS
		${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( QT_DIR ${CMAKE_BINARY_DIR}/${proj} )
SET( QT_INCLUDE_DIR ${CMAKE_BINARY_DIR}/${proj}/include )
SET( QT_LIBRARY_DIR ${CMAKE_BINARY_DIR}/${proj}/lib )

mark_as_superbuild(
	VARS
		QT_DIR:PATH
		QT_INCLUDE_DIR:PATH
		QT_LIBRARY_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "QT_DIR: ${QT_DIR}" )
ExternalProject_Message( ${proj} "QT_INCLUDE_DIR: ${QT_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "QT_LIBRARY_DIR: ${QT_LIBRARY_DIR}" )
### --- End binary information
