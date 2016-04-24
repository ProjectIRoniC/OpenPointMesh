
# Install the XnLib library
MESSAGE( STATUS "installing XnLib library..." )
EXECUTE_PROCESS( COMMAND ${CMAKE_COMMAND}
					-E copy_directory ${XNLIB_BIN_DIR} ${INSTALL_DIR}/lib
					RESULT_VARIABLE install_result
)

IF( NOT "${install_result}" STREQUAL "0" )
	MESSAGE( STATUS "install XnLib Failed!!!" )
	MESSAGE( FATAL_ERROR "install_result='${install_result}'" )
ENDIF()

# Install the XnLib includes
MESSAGE( STATUS "installing XnLib includes..." )
EXECUTE_PROCESS( COMMAND ${CMAKE_COMMAND}
					-E copy_directory ${XNLIB_INCLUDE_DIR} ${INSTALL_DIR}/include
					RESULT_VARIABLE install_result
)

IF( NOT "${install_result}" STREQUAL "0" )
	MESSAGE( STATUS "install XnLib Failed!!!" )
	MESSAGE( FATAL_ERROR "install_result='${install_result}'" )
ENDIF()

# Copy the OpenNI2 bin directory
MESSAGE( STATUS "installing OpenNI2 bin..." )
EXECUTE_PROCESS( COMMAND ${CMAKE_COMMAND}
					-E copy_directory ${OPENNI2_LIB_BIN_DIR} ${INSTALL_DIR}/lib
					RESULT_VARIABLE install_result
)

IF( NOT "${install_result}" STREQUAL "0" )
	MESSAGE( STATUS "install OpenNI2 bin Failed!!!" )
	MESSAGE( FATAL_ERROR "install_result='${install_result}'" )
ENDIF()

# Install the OpenNI2 includes
MESSAGE( STATUS "installing OpenNI2 includes..." )
EXECUTE_PROCESS( COMMAND ${CMAKE_COMMAND}
					-E copy_directory ${OPENNI2_LIB_INCLUDE_DIR} ${INSTALL_DIR}/include
					RESULT_VARIABLE install_result
)

IF( NOT "${install_result}" STREQUAL "0" )
	MESSAGE( STATUS "install OpenNI2 includes Failed!!!" )
	MESSAGE( FATAL_ERROR "install_result='${install_result}'" )
ENDIF()

# Install the PS1080 includes
MESSAGE( STATUS "installing PS1080 includes..." )
EXECUTE_PROCESS( COMMAND ${CMAKE_COMMAND}
					-E copy_directory ${PS1080_INCLUDE_DIR} ${INSTALL_DIR}/include
					RESULT_VARIABLE install_result
)

IF( NOT "${install_result}" STREQUAL "0" )
	MESSAGE( STATUS "install PS1080 includes Failed!!!" )
	MESSAGE( FATAL_ERROR "install_result='${install_result}'" )
ENDIF()

RETURN( ${install_result} )
