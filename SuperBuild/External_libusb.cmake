# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName libusb ) # The find_package known name
SET( proj        libusb ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES "" )

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj} )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( ${proj}_CONFIGURE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_libusb_configurelibusb.cmake )
SET( ${proj}_CONFIGURE_COMMAND
	${CMAKE_COMMAND}
	# CMake Build ARGS
	-DLIBUSB_C_FLAGS:STRING=${NONCMAKE_EP_COMMON_C_FLAGS}
	-DLIBUSB_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
	-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
	-DINSTALL_DIR:PATH=${${proj}_INSTALL_DIR}
	-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
	-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
	# Use the configure script
	-P ${${proj}_CONFIGURE_SCRIPT}
)

# Download tar source when possible to speed up build time
#SET( ${proj}_URL https://github.com/libusb/libusb/archive/v1.0.20.tar.gz )
#SET( ${proj}_MD5 133eaa6c73cd4d4da4a1c58bb76b45cf )
SET( ${proj}_REPOSITORY "${git_protocol}://github.com/libusb/libusb" )
SET( ${proj}_GIT_TAG "master" )
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
#	URL					${${proj}_URL}
#	URL_MD5				${${proj}_MD5}
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

### --- Set binary information
SET( LIBUSB_DIR ${${proj}_INSTALL_DIR} )
SET( LIBUSB_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( LIBUSB_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( LIBUSB_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )
	
mark_as_superbuild(
	VARS
		LIBUSB_DIR:PATH
		LIBUSB_BUILD_DIR:PATH
		LIBUSB_INCLUDE_DIR:PATH
		LIBUSB_LIBRARY_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "LIBUSB_DIR: ${LIBUSB_DIR}" )
ExternalProject_Message( ${proj} "LIBUSB_BUILD_DIR: ${LIBUSB_BUILD_DIR}" )
ExternalProject_Message( ${proj} "LIBUSB_INCLUDE_DIR: ${LIBUSB_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "LIBUSB_LIBRARY_DIR: ${LIBUSB_LIBRARY_DIR}" )
### --- End binary information

# Force rerun of autoconf
#ExternalProject_Add_Step( ${proj} "run reautoconf"
#	COMMAND ${CMAKE_COMMAND}
#		-DAUTORECONF_IN_DIR:PATH=${${proj}_SOURCE_DIR}
#		-P ${RUN_AUTORECONF_SCRIPT}
#	
#	DEPENDEES download
#	DEPENDERS configure
#)
