
# Set directories
SET( ONIFILE_SOURCE_DIR ${SOURCE_DIR}/Source/Drivers/OniFile )

# Set compile and link flags
SET( ONIFILE_CXX_FLAGS "${EP_NONCMAKE_COMMON_CXX_FLAGS} -I${XNLIB_INCLUDE_DIR} -I${OPENNI2_LIB_INCLUDE_DIR} -I${ONIFILE_FORMATS_DIR}" )
SET( ONIFILE_LD_FLAGS "${CMAKE_EXE_LINKER_CXX_FLAGS} -L${XNLIB_BIN_DIR} -l${XNLIB_LIBRARY_NAME} -L${OPENNI2_LIB_BIN_DIR}" )

# Add flags for Position Independent Code
IF( CMAKE_POSITION_INDEPENDENT_CODE )
	SET( ONIFILE_CXX_FLAGS "${ONIFILE_CXX_FLAGS} -fPIC" )
ENDIF()

# Create the build command arguments list
SET( ONIFILE_BUILD_ARGS CFLAGS=${ONIFILE_CXX_FLAGS} LDFLAGS=${ONIFILE_LD_FLAGS} ALLOW_WARNINGS=1 )

# Select Shared or Static Libraries
IF( BUILD_SHARED_LIBS )
	LIST( APPEND ONIFILE_BUILD_ARGS LIB_NAME=${LIBRARY_NAME} )
ELSE()
	LIST( APPEND ONIFILE_BUILD_ARGS SLIB_NAME=${LIBRARY_NAME} )
ENDIF()

# If needed set debug information
IF( CMAKE_BUILD_TYPE MATCHES "Debug" )
	LIST( APPEND ONIFILE_BUILD_ARGS CFG=Debug )
ENDIF()

# Run the build command
EXECUTE_PROCESS( COMMAND make ${ONIFILE_BUILD_ARGS}
					WORKING_DIRECTORY ${ONIFILE_SOURCE_DIR} RESULT_VARIABLE build_result
)

IF( NOT "${build_result}" STREQUAL "0" )
	MESSAGE( STATUS "build OniFile Failed!!!" )
	MESSAGE( FATAL_ERROR "build_result='${build_result}'" )
ENDIF()

RETURN( ${build_result} )
