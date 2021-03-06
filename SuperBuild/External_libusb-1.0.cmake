# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName libusb-1.0 ) # The find_package known name
SET( proj        libusb-1.0 ) # The local name
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
SET( ${proj}_LIBRARY_INSTALL_DIR ${${proj}_INSTALL_DIR}/lib )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( ${proj}_CONFIGURE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_libusb-1.0_configurelibusb-1.0.cmake )
SET( ${proj}_CONFIGURE_COMMAND
	${CMAKE_COMMAND}
		# CMake Build ARGS
		-DEP_NONCMAKE_COMMON_C_FLAGS:STRING=${EP_NONCMAKE_COMMON_C_FLAGS}
		-DCMAKE_EXE_LINKER_C_FLAGS:STRING=${CMAKE_EXE_LINKER_C_FLAGS}
		-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
		-DINSTALL_DIR:PATH=${${proj}_INSTALL_DIR}
		-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
		-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
		# Use the configure script
		-P ${${proj}_CONFIGURE_SCRIPT}
)

# Download tar source when possible to speed up build time
#SET( ${proj}_URL https://github.com/libusb/libusb/archive/v1.0.20.tar.gz )
#SET( ${proj}_MD5 133eaa6c73cd4d4da4a1c58bb76b45cf )
SET( ${proj}_REPOSITORY "${git_protocol}://github.com/libusb/libusb.git" )
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

# Force rerun of autoconf
ExternalProject_Add_Step( ${proj} run_reautoconf
	COMMAND ${CMAKE_COMMAND}
		-DAUTORECONF_IN_DIR:PATH=${${proj}_SOURCE_DIR}
		-P ${RUN_AUTORECONF_SCRIPT}
	
	DEPENDEES download
	DEPENDERS configure
)

### --- Set binary information
SET( LIBUSB_1_DIR ${${proj}_INSTALL_DIR} )
SET( LIBUSB_1_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( LIBUSB_1_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( LIBUSB_1_LIBRARY_DIR ${${proj}_LIBRARY_INSTALL_DIR} )
SET( LIBUSB_1_LIBRARY_NAME usb-1.0 )
	
mark_as_superbuild(
	VARS
		LIBUSB_1_DIR:PATH
		LIBUSB_1_BUILD_DIR:PATH
		LIBUSB_1_INCLUDE_DIR:PATH
		LIBUSB_1_LIBRARY_DIR:PATH
		LIBUSB_1_LIBRARY_NAME:STRING
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "LIBUSB_1_DIR: ${LIBUSB_1_DIR}" )
ExternalProject_Message( ${proj} "LIBUSB_1_BUILD_DIR: ${LIBUSB_1_BUILD_DIR}" )
ExternalProject_Message( ${proj} "LIBUSB_1_INCLUDE_DIR: ${LIBUSB_1_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "LIBUSB_1_LIBRARY_DIR: ${LIBUSB_1_LIBRARY_DIR}" )
### --- End binary information
