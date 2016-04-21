
# Set directories
SET( PSLINK_SOURCE_DIR ${SOURCE_DIR}/Source/Drivers/PSLink )
SET( XNLINK_PROTOCOL_DIR ${PSLINK_SOURCE_DIR}/Protocols/XnLinkProto )
SET( LINK_PROTOCOL_DIR ${PSLINK_SOURCE_DIR}/LinkProtoLib )

# Set compile and link flags
SET( PSLINK_CXX_FLAGS "${EP_NONCMAKE_COMMON_CXX_FLAGS} -I${PSLINK_SOURCE_DIR} -I${LIBUSB_1_INCLUDE_DIR} -I${XNLIB_INCLUDE_DIR} -I${OPENNI2_LIB_INCLUDE_DIR} -I${DEPTHUTILS_INCLUDE_DIR} -I${XNLINK_PROTOCOL_DIR} -I${LINK_PROTOCOL_DIR}" )
SET( PSLINK_LD_FLAGS "${CMAKE_EXE_LINKER_CXX_FLAGS} -L${LIBUSB_1_LIBRARY_DIR} -L${XNLIB_BIN_DIR} -L${OPENNI2_LIB_BIN_DIR}" )

# Add flags for Position Independent Code
IF( CMAKE_POSITION_INDEPENDENT_CODE )
	SET( PSLINK_CXX_FLAGS "${PSLINK_CXX_FLAGS} -fPIC" )
ENDIF()

# Create the build command arguments list
SET( PSLINK_BUILD_ARGS CFLAGS=${PSLINK_CXX_FLAGS} LDFLAGS=${PSLINK_LD_FLAGS} ALLOW_WARNINGS=1 )

# If needed set debug information
IF( CMAKE_BUILD_TYPE MATCHES "Debug" )
	LIST( APPEND PSLINK_BUILD_ARGS CFG=Debug )
ENDIF()

# Run the build command
EXECUTE_PROCESS( COMMAND make ${PSLINK_BUILD_ARGS}
			WORKING_DIRECTORY ${PSLINK_SOURCE_DIR} RESULT_VARIABLE build_result
)

IF( NOT "${build_result}" STREQUAL "0" )
	MESSAGE( STATUS "build PSLink Failed!!!" )
	MESSAGE( FATAL_ERROR "build_result='${build_result}'" )
ENDIF()

RETURN( ${build_result} )
