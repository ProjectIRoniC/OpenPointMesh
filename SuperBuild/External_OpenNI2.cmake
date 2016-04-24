# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName OpenNI2 ) # The find_package known name
SET( proj        OpenNI2 ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE(FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})")
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	libusb-1.0
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj} )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )
SET( XNLIB_INCLUDE_DIR ${${proj}_SOURCE_DIR}/ThirdParty/PSCommon/XnLib/Include )
SET( XNLIB_BIN_DIR ${${proj}_SOURCE_DIR}/ThirdParty/PSCommon/XnLib/Bin/x64-Release )
SET( OPENNI2_LIB_INCLUDE_DIR ${${proj}_SOURCE_DIR}/Include )
SET( OPENNI2_LIB_BIN_DIR ${${proj}_SOURCE_DIR}/Bin/x64-Release )
SET( DEPTHUTILS_INCLUDE_DIR ${${proj}_SOURCE_DIR}/Source/DepthUtils )
SET( INTERNAL_JPEG_INCLUDE_DIR ${${proj}_SOURCE_DIR}/ThirdParty/LibJPEG )
SET( ONIFILE_FORMATS_DIR ${${proj}_SOURCE_DIR}/Source/Drivers/OniFile/Formats )
SET( PS1080_INCLUDE_DIR ${${proj}_SOURCE_DIR}/Source/Drivers/PS1080/Include )

IF( CMAKE_BUILD_TYPE MATCHES "Debug" )
	SET( OPENNI2_LIB_BIN_DIR ${SOURCE_DIR}/Bin/x64-Debug )
ENDIF()

### --- Project specific additions here
SET( ${proj}_INSTALL_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_OpenNI2_installopenni2.cmake )
SET( ${proj}_INSTALL_COMMAND
	${CMAKE_COMMAND}
		# CMake ARGS
		-DINSTALL_DIR:PATH=${${proj}_INSTALL_DIR}
		# OpenNI2 ARGS
		-DOPENNI2_LIB_INCLUDE_DIR:PATH=${OPENNI2_LIB_INCLUDE_DIR}
		-DOPENNI2_LIB_BIN_DIR:PATH=${OPENNI2_LIB_BIN_DIR}
		# PS1080 ARGS
		-DPS1080_INCLUDE_DIR:PATH=${PS1080_INCLUDE_DIR}
		# XnLib ARGS
		-DXNLIB_INCLUDE_DIR:PATH=${XNLIB_INCLUDE_DIR}
		-DXNLIB_BIN_DIR:PATH=${XNLIB_BIN_DIR}
		# Use the install script
		-P ${${proj}_INSTALL_SCRIPT}
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/occipital/OpenNI2/archive/v2.2.0-debian.tar.gz )
SET( ${proj}_MD5 bdb95be379150c6bd0433f8a6862ee7f )
#SET( ${proj}_REPOSITORY "${git_protocol}://github.com/occipital/OpenNI2.git" )
#SET( ${proj}_GIT_TAG v2.2.0-debian )  # 2.2.0-debian
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL					${${proj}_URL}
	URL_MD5				${${proj}_MD5}
	# GIT_REPOSITORY	${${proj}_REPOSITORY}
	# GIT_TAG 			${${proj}_GIT_TAG}
	SOURCE_DIR			${${proj}_SOURCE_DIR}
	BUILD_IN_SOURCE		1
	INSTALL_DIR			${${proj}_INSTALL_DIR}
	LOG_DOWNLOAD		${EP_LOG_DOWNLOAD}
	LOG_UPDATE			${EP_LOG_UPDATE}
	LOG_CONFIGURE		${EP_LOG_CONFIGURE}
	LOG_BUILD			${EP_LOG_BUILD}
	LOG_TEST			${EP_LOG_TEST}
	LOG_INSTALL			${EP_LOG_INSTALL}
	CONFIGURE_COMMAND	"" # All configuration done manually in build scripts
	BUILD_COMMAND		"" # See below for build info
	INSTALL_COMMAND		${${proj}_INSTALL_COMMAND}
	DEPENDS 			${${proj}_DEPENDENCIES}
)

# OpenNI2 has a top level makefile that expects all dependencies to be installed locally,
# since we build some of the dependencies we need to specify C/C++ flags manually to
# include our sources. In order to avoid overlapping include directories we are building
# each library and driver separately as added steps
# Build XnLib first since all the other binaries depend on it
SET( ${proj}_BUILD_XNLIB_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_OpenNI2_buildxnlib.cmake )
SET( XNLIB_LIBRARY_NAME "XnLib" ) # We are using the name OpenNI2 makefiles are expecting
ExternalProject_Add_Step( ${proj} build_XnLib
	COMMAND ${CMAKE_COMMAND}
		# CMake ARGS
		-DEP_NONCMAKE_COMMON_CXX_FLAGS:STRING=${EP_NONCMAKE_COMMON_CXX_FLAGS}
		-DCMAKE_EXE_LINKER_CXX_FLAGS:STRING=${CMAKE_EXE_LINKER_CXX_FLAGS}
		-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
		-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
		-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
		# XnLib ARGS
		-DLIBRARY_NAME:STRING=${XNLIB_LIBRARY_NAME}
		-DXNLIB_INCLUDE_DIR:PATH=${XNLIB_INCLUDE_DIR}
		-DXNLIB_BIN_DIR:PATH=${XNLIB_BIN_DIR}
		# LibUSB ARGS
		-DLIBUSB_1_INCLUDE_DIR:PATH=${LIBUSB_1_INCLUDE_DIR}
		-DLIBUSB_1_LIBRARY_DIR:PATH=${LIBUSB_1_LIBRARY_DIR}
		# Use the build script
		-P ${${proj}_BUILD_XNLIB_SCRIPT}

	# Does not depend on any libraries
	DEPENDEES build
	DEPENDERS install
)

# Build OpenNI2
SET( ${proj}_BUILD_OPENNI2_LIB_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_OpenNI2_buildopenni2.cmake )
SET( OPENNI2_LIBRARY_NAME OpenNI2 ) # We are using the name OpenNI2 makefiles are expecting
ExternalProject_Add_Step( ${proj} build_OpenNI2
	COMMAND ${CMAKE_COMMAND}
		# CMake ARGS
		-DEP_NONCMAKE_COMMON_CXX_FLAGS:STRING=${EP_NONCMAKE_COMMON_CXX_FLAGS}
		-DCMAKE_EXE_LINKER_CXX_FLAGS:STRING=${CMAKE_EXE_LINKER_CXX_FLAGS}
		-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
		-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
		-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
		# OpenNI2 ARGS
		-DLIBRARY_NAME:STRING=${OPENNI2_LIBRARY_NAME}
		-DOPENNI2_LIB_INCLUDE_DIR:PATH=${OPENNI2_LIB_INCLUDE_DIR}
		# OniFile ARGS
		-DONIFILE_FORMATS_DIR:PATH=${ONIFILE_FORMATS_DIR}
		# Internal JPEG ARGS
		-DINTERNAL_JPEG_INCLUDE_DIR:PATH=${INTERNAL_JPEG_INCLUDE_DIR}
		# XnLib ARGS
		-DXNLIB_INCLUDE_DIR:PATH=${XNLIB_INCLUDE_DIR}
		-DXNLIB_BIN_DIR:PATH=${XNLIB_BIN_DIR}
		# Use the build script
		-P ${${proj}_BUILD_OPENNI2_LIB_SCRIPT}

	# Depends on XnLib
	DEPENDEES build_XnLib
	DEPENDERS install
)

# Build DepthUtils
SET( ${proj}_BUILD_DEPTHUTILS_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_OpenNI2_builddepthutils.cmake )
SET( DEPTHUTILS_LIBRARY_NAME DepthUtils ) # We are using the name OpenNI2 makefiles are expecting
ExternalProject_Add_Step( ${proj} build_DepthUtils
	COMMAND ${CMAKE_COMMAND}
		# CMake ARGS
		-DEP_NONCMAKE_COMMON_CXX_FLAGS:STRING=${EP_NONCMAKE_COMMON_CXX_FLAGS}
		-DCMAKE_EXE_LINKER_CXX_FLAGS:STRING=${CMAKE_EXE_LINKER_CXX_FLAGS}
		-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
		-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
		-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
		# DepthUtils ARGS
		-DLIBRARY_NAME:STRING=${DEPTHUTILS_LIBRARY_NAME}
		-DOPENNI2_LIB_INCLUDE_DIR:PATH=${OPENNI2_LIB_INCLUDE_DIR}
		# XnLib ARGS
		-DXNLIB_INCLUDE_DIR:PATH=${XNLIB_INCLUDE_DIR}
		-DXNLIB_BIN_DIR:PATH=${XNLIB_BIN_DIR}
		# Use the build script
		-P ${${proj}_BUILD_DEPTHUTILS_SCRIPT}

	# Depends on XnLib
	DEPENDEES build_XnLib
	DEPENDERS install
)

# Build DummyDevice
SET( ${proj}_BUILD_DUMMYDEVICE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_OpenNI2_builddummydevice.cmake )
SET( DUMMYDEVICE_LIBRARY_NAME DummyDevice ) # We are using the name OpenNI2 makefiles are expecting
ExternalProject_Add_Step( ${proj} build_DummyDevice
	COMMAND ${CMAKE_COMMAND}
		# CMake ARGS
		-DEP_NONCMAKE_COMMON_CXX_FLAGS:STRING=${EP_NONCMAKE_COMMON_CXX_FLAGS}
		-DCMAKE_EXE_LINKER_CXX_FLAGS:STRING=${CMAKE_EXE_LINKER_CXX_FLAGS}
		-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
		-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
		-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
		# DummyDevice ARGS
		-DLIBRARY_NAME:STRING=${DUMMYDEVICE_LIBRARY_NAME}
		# OpenNI2 ARGS
		-DOPENNI2_LIB_INCLUDE_DIR:PATH=${OPENNI2_LIB_INCLUDE_DIR}
		-DOPENNI2_LIB_BIN_DIR:PATH=${OPENNI2_LIB_BIN_DIR}
		# XnLib ARGS
		-DXNLIB_INCLUDE_DIR:PATH=${XNLIB_INCLUDE_DIR}
		-DXNLIB_BIN_DIR:PATH=${XNLIB_BIN_DIR}
		# Use the build script
		-P ${${proj}_BUILD_DUMMYDEVICE_SCRIPT}

	# Depends on XnLib and OpenNI2
	DEPENDEES build_XnLib build_OpenNI2
	DEPENDERS install
)

# Build PS1080
SET( ${proj}_BUILD_PS1080_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_OpenNI2_buildps1080.cmake )
SET( PS1080_LIBRARY_NAME PS1080 ) # We are using the name OpenNI2 makefiles are expecting
ExternalProject_Add_Step( ${proj} build_PS1080
	COMMAND ${CMAKE_COMMAND}
		# CMake ARGS
		-DEP_NONCMAKE_COMMON_CXX_FLAGS:STRING=${EP_NONCMAKE_COMMON_CXX_FLAGS}
		-DCMAKE_EXE_LINKER_CXX_FLAGS:STRING=${CMAKE_EXE_LINKER_CXX_FLAGS}
		-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
		-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
		-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
		# PS1080 ARGS
		-DLIBRARY_NAME:STRING=${PS1080_LIBRARY_NAME}
		-DPS1080_INCLUDE_DIR:PATH=${PS1080_INCLUDE_DIR}
		# DepthUtils ARGS
		-DDEPTHUTILS_INCLUDE_DIR:PATH=${DEPTHUTILS_INCLUDE_DIR}
		# Internal JPEG ARGS
		-DINTERNAL_JPEG_INCLUDE_DIR:PATH=${INTERNAL_JPEG_INCLUDE_DIR}
		# LibUSB ARGS
		-DLIBUSB_1_INCLUDE_DIR:PATH=${LIBUSB_1_INCLUDE_DIR}
		-DLIBUSB_1_LIBRARY_DIR:PATH=${LIBUSB_1_LIBRARY_DIR}
		# OpenNI2 ARGS
		-DOPENNI2_LIB_INCLUDE_DIR:PATH=${OPENNI2_LIB_INCLUDE_DIR}
		-DOPENNI2_LIB_BIN_DIR:PATH=${OPENNI2_LIB_BIN_DIR}
		# XnLib ARGS
		-DXNLIB_INCLUDE_DIR:PATH=${XNLIB_INCLUDE_DIR}
		-DXNLIB_BIN_DIR:PATH=${XNLIB_BIN_DIR}
		# Use the build script
		-P ${${proj}_BUILD_PS1080_SCRIPT}

	# Depends on XnLib, OpenNI2 and DepthUtils
	DEPENDEES build_XnLib build_OpenNI2 build_DepthUtils
	DEPENDERS install
)

# Build PSLink
SET( ${proj}_BUILD_PSLINK_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_OpenNI2_buildpslink.cmake )
SET( PSLINK_LIBRARY_NAME PSLink ) # We are using the name OpenNI2 makefiles are expecting
ExternalProject_Add_Step( ${proj} build_PSLink
	COMMAND ${CMAKE_COMMAND}
		# CMake ARGS
		-DEP_NONCMAKE_COMMON_CXX_FLAGS:STRING=${EP_NONCMAKE_COMMON_CXX_FLAGS}
		-DCMAKE_EXE_LINKER_CXX_FLAGS:STRING=${CMAKE_EXE_LINKER_CXX_FLAGS}
		-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
		-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
		-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
		# PSLink ARGS
		-DLIBRARY_NAME:STRING=${PSLINK_LIBRARY_NAME}
		# DepthUtils ARGS
		-DDEPTHUTILS_INCLUDE_DIR:PATH=${DEPTHUTILS_INCLUDE_DIR}
		# LibUSB ARGS
		-DLIBUSB_1_INCLUDE_DIR:PATH=${LIBUSB_1_INCLUDE_DIR}
		-DLIBUSB_1_LIBRARY_DIR:PATH=${LIBUSB_1_LIBRARY_DIR}
		# OpenNI2 ARGS
		-DOPENNI2_LIB_INCLUDE_DIR:PATH=${OPENNI2_LIB_INCLUDE_DIR}
		-DOPENNI2_LIB_BIN_DIR:PATH=${OPENNI2_LIB_BIN_DIR}
		# XnLib ARGS
		-DXNLIB_INCLUDE_DIR:PATH=${XNLIB_INCLUDE_DIR}
		-DXNLIB_BIN_DIR:PATH=${XNLIB_BIN_DIR}
		# Use the build script
		-P ${${proj}_BUILD_PSLINK_SCRIPT}

	# Depends on XnLib, OpenNI2 and DepthUtils
	DEPENDEES build_XnLib build_OpenNI2 build_DepthUtils
	DEPENDERS install
)

# Build OniFile
SET( ${proj}_BUILD_ONIFILE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/External_OpenNI2_buildonifile.cmake )
SET( ONIFILE_LIBRARY_NAME OniFile ) # We are using the name OpenNI2 makefiles are expecting
ExternalProject_Add_Step( ${proj} build_OniFile
	COMMAND ${CMAKE_COMMAND}
		# CMake ARGS
		-DEP_NONCMAKE_COMMON_CXX_FLAGS:STRING=${EP_NONCMAKE_COMMON_CXX_FLAGS}
		-DCMAKE_EXE_LINKER_CXX_FLAGS:STRING=${CMAKE_EXE_LINKER_CXX_FLAGS}
		-DSOURCE_DIR:PATH=${${proj}_SOURCE_DIR}
		-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
		-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
		# OniFile ARGS
		-DLIBRARY_NAME:STRING=${ONIFILE_LIBRARY_NAME}
		# Internal JPEG ARGS
		-DINTERNAL_JPEG_INCLUDE_DIR:PATH=${INTERNAL_JPEG_INCLUDE_DIR}
		# OniFile ARGS
		-DONIFILE_FORMATS_DIR:PATH=${ONIFILE_FORMATS_DIR}
		# OpenNI2 ARGS
		-DOPENNI2_LIB_INCLUDE_DIR:PATH=${OPENNI2_LIB_INCLUDE_DIR}
		-DOPENNI2_LIB_BIN_DIR:PATH=${OPENNI2_LIB_BIN_DIR}
		# XnLib ARGS
		-DXNLIB_INCLUDE_DIR:PATH=${XNLIB_INCLUDE_DIR}
		-DXNLIB_BIN_DIR:PATH=${XNLIB_BIN_DIR}
		# Use the build script
		-P ${${proj}_BUILD_ONIFILE_SCRIPT}

	# Depends on XnLib and OpenNI2
	DEPENDEES build_XnLib build_OpenNI2
	DEPENDERS install
)

### --- Set binary information
SET( OPENNI2_DIR ${${proj}_INSTALL_DIR} )
SET( OPENNI2_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( OPENNI2_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( OPENNI2_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )

mark_as_superbuild(
  VARS
    OPENNI2_DIR:PATH
	OPENNI2_BUILD_DIR:PATH
	OPENNI2_INCLUDE_DIR:PATH
	OPENNI2_LIBRARY_DIR:PATH
  LABELS
     "FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "OPENNI2_DIR: ${OPENNI2_DIR}" )
ExternalProject_Message( ${proj} "OPENNI2_BUILD_DIR: ${OPENNI2_BUILD_DIR}" )
ExternalProject_Message( ${proj} "OPENNI2_INCLUDE_DIR: ${OPENNI2_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "OPENNI2_LIBRARY_DIR: ${OPENNI2_LIBRARY_DIR}" )
### --- End binary information
