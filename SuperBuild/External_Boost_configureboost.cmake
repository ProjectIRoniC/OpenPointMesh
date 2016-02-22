
# bootstrap.bat has no options, the options are given to ./b2 when BUILDING
# see External_Boost_buildboost.cmake for configure options
SET( BOOST_CONFIGURE_COMMAND
	bootstrap.bat
)

# Set explicitly to mingw since boost will assume MSVC on WIN32
IF( WIN32 )
	IF( CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
		LIST( APPEND BOOST_CONFIGURE_COMMAND
			mingw
		)
	ENDIF()
ENDIF()

EXECUTE_PROCESS( COMMAND ${BOOST_CONFIGURE_COMMAND}
			WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE boostrap_result )

RETURN( ${bootstrap_result} )
