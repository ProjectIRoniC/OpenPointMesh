
# Set directories
SET( DEPTHUTILS_SOURCE_DIR ${SOURCE_DIR}/Source/DepthUtils )

# Set compile and link flags
SET( DEPTHUTILS_CXX_FLAGS "${EP_NONCMAKE_COMMON_CXX_FLAGS} -I${XNLIB_INCLUDE_DIR} -I${OPENNI2_LIB_INCLUDE_DIR}" )
SET( DEPTHUTILS_LD_FLAGS "${CMAKE_EXE_LINKER_CXX_FLAGS} -L${XNLIB_BIN_DIR}" )

# Add flags for Position Independent Code
IF( CMAKE_POSITION_INDEPENDENT_CODE )
	SET( DEPTHUTILS_CXX_FLAGS "${DEPTHUTILS_CXX_FLAGS} -fPIC" )
ENDIF()

# Create the build command arguments list
SET( DEPTHUTILS_BUILD_ARGS CFLAGS=${DEPTHUTILS_CXX_FLAGS} LDFLAGS=${DEPTHUTILS_LD_FLAGS} ALLOW_WARNINGS=1 )

# Select Shared or Static Libraries
IF( BUILD_SHARED_LIBS )
	LIST( APPEND DEPTHUTILS_BUILD_ARGS LIB_NAME=${LIBRARY_NAME} )
ELSE()
	LIST( APPEND DEPTHUTILS_BUILD_ARGS SLIB_NAME=${LIBRARY_NAME} )
ENDIF()

# If needed set debug information
IF( CMAKE_BUILD_TYPE MATCHES "Debug" )
	LIST( APPEND DEPTHUTILS_BUILD_ARGS CFG=Debug )
ENDIF()

# Run the build command
EXECUTE_PROCESS( COMMAND make ${DEPTHUTILS_BUILD_ARGS}
					WORKING_DIRECTORY ${DEPTHUTILS_SOURCE_DIR} RESULT_VARIABLE build_result
)

IF( NOT "${build_result}" STREQUAL "0" )
	MESSAGE( STATUS "build DepthUtils Failed!!!" )
	MESSAGE( FATAL_ERROR "build_result='${build_result}'" )
ENDIF()

RETURN( ${build_result} )
