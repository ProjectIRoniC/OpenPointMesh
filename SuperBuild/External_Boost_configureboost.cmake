#
# Follow the boost suggestions
#

IF( WIN32 ) # bootstrap.bat has no options, the options are given to ./b2 when BUILDING: see buildboost.cmake
	IF( CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
		EXECUTE_PROCESS( COMMAND bootstrap.bat mingw
			WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE boostrap_result )
	ELSE()
		EXECUTE_PROCESS( COMMAND bootstrap.bat
			WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE boostrap_result )
	ENDIF()
ELSE( WIN32 )
	EXECUTE_PROCESS( COMMAND ./bootstrap.sh --prefix=${BOOST_INSTALL_DIR}
		--with-libraries=system,thread,program_options,log,math
## Needed for UKF system,thread,
## Needed for DTIProcess program_options locale
##    --without-libraries=atomic,chrono,context,date_time,exception,filesystem,graph,graph_parallel,iostreams,log,math,mpi,python,random,regex,serialization,signals,test,timer,wave
		WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE boostrap_result )
### NOTE:  --with-libraries= is purposefull left blank to avoid building
###        any of the unnecessary boost libraries.  ANTS only needs
###        the header-only components of boost!
ENDIF( WIN32 )

RETURN( ${bootstrap_result} )
