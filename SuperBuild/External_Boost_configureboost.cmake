
# bootstrap.bat/sh has no options, the options are given to ./b2 when BUILDING
# see External_Boost_buildboost.cmake for configure options
IF( WIN32 )
	SET( BOOST_CONFIGURE_COMMAND bootstrap.bat )
ELSE( )
	SET( BOOST_CONFIGURE_COMMAND ./bootstrap.sh )
ENDIF()

# Set explicitly to mingw since boost will assume MSVC on WIN32
IF( WIN32 )
	IF( CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
		LIST( APPEND BOOST_CONFIGURE_COMMAND
			mingw
		)
	ENDIF()
ENDIF()

EXECUTE_PROCESS( COMMAND ${BOOST_CONFIGURE_COMMAND}
			WORKING_DIRECTORY ${SOURCE_DIR} RESULT_VARIABLE bootstrap_result )

IF( NOT "${bootstrap_result}" STREQUAL "0" )
	MESSAGE( STATUS "Boost bootstrap.bat Failed!!!" )
	MESSAGE( FATAL_ERROR "bootstrap_result='${bootstrap_result}'" )
ENDIF()
			
RETURN( ${bootstrap_result} )
