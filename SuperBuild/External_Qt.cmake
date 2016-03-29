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
	Freetype
	FontConfig
	JPEG
	MNG
	PNG
	TIFF
	ZLIB
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj} )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj} )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( ${proj}_CONFIGURE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_Qt_configureqt.cmake )
SET( ${proj}_CONFIGURE_COMMAND
	# CMake ARGS
	${CMAKE_COMMAND}
	-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
	-DINSTALL_DIR:PATH=${${proj}_INSTALL_DIR}
	-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
	# FontConfig ARGS
	-DFONTCONFIG_INCLUDE_DIR:PATH=${FONTCONFIG_INCLUDE_DIR}
	-DFONTCONFIG_LIBRARY_DIR:PATH=${FONTCONFIG_LIBRARY_DIR}
	# FreeType ARGS
	-DFREETYPE_INCLUDE_DIR:PATH=${FREETYPE_INCLUDE_DIR}
	-DFREETYPE_LIBRARY_DIR:PATH=${FREETYPE_LIBRARY_DIR}
	# JPEG ARGS
	-DJPEG_INCLUDE_DIR:PATH=${JPEG_INCLUDE_DIR}
	-DJPEG_LIBRARY_DIR:PATH=${JPEG_LIBRARY_DIR}
	# LCMS ARGS
	-DLCMS_LIBRARY_DIR:PATH=${LCMS_LIBRARY_DIR}
	-DLCMS_LIBRARY_NAME:STRING=${LCMS_LIBRARY_NAME}
	# MNG ARGS
	-DMNG_INCLUDE_DIR:PATH=${MNG_INCLUDE_DIR}
	-DMNG_LIBRARY_DIR:PATH=${MNG_LIBRARY_DIR}
	# PNG ARGS
	-DPNG_INCLUDE_DIR:PATH=${PNG_INCLUDE_DIR}
	-DPNG_LIBRARY_DIR:PATH=${PNG_LIBRARY_DIR}
	# TIFF ARGS
	-DTIFF_INCLUDE_DIR:PATH=${TIFF_INCLUDE_DIR}
	-DTIFF_LIBRARY_DIR:PATH=${TIFF_LIBRARY_DIR}
	# ZLIB ARGS
	-DZLIB_INCLUDE_DIR:PATH=${ZLIB_INCLUDE_DIR}
	-DZLIB_LIBRARY_DIR:PATH=${ZLIB_LIBRARY_DIR}
	# Use the configure script
	-P ${${proj}_CONFIGURE_SCRIPT}
)

SET( ${proj}_URL https://download.qt.io/archive/qt/4.8/4.8.6/qt-everywhere-opensource-src-4.8.6.tar.gz )
SET( ${proj}_MD5 2edbe4d6c2eff33ef91732602f3518eb )
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL					${${proj}_URL}
	URL_MD5				${${proj}_MD5}
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
	INSTALL_COMMAND		""
	DEPENDS 			${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( QT_DIR ${${proj}_INSTALL_DIR} )
SET( QT_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( QT_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( QT_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )
SET( QT_QMAKE_EXECUTABLE ${${proj}_INSTALL_DIR}/bin/qmake )

mark_as_superbuild(
	VARS
		QT_DIR:PATH
		QT_BUILD_DIR:PATH
		QT_INCLUDE_DIR:PATH
		QT_LIBRARY_DIR:PATH
		QT_QMAKE_EXECUTABLE:FILEPATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "QT_DIR: ${QT_DIR}" )
ExternalProject_Message( ${proj} "QT_BUILD_DIR: ${QT_BUILD_DIR}" )
ExternalProject_Message( ${proj} "QT_INCLUDE_DIR: ${QT_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "QT_LIBRARY_DIR: ${QT_LIBRARY_DIR}" )
ExternalProject_Message( ${proj} "QT_QMAKE_EXECUTABLE: ${QT_QMAKE_EXECUTABLE}" )
### --- End binary information
