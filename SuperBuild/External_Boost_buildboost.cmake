#
# Follow the boost suggestions
#

IF( WIN32 )
	SET( XXX "
		# Needed
			--with-atomic	
			--with-chrono 
			--with-context 
			--with-date_time 
			--with-exception 
			--with-filesystem
			--with-format
			--with-graph 
			--with-graph_parallel 
			--with-iostreams 
			--with-locale
			--with-log 
			--with-math 
			--with-mpi 
			--with-program_options
			--with-python 
			--with-random 
			--with-regex 
			--with-serialization 
			--with-signals
			--with-system
			--with-thread
			--with-timer 
			--with-wave
		# Not Needed
			--without-test 
	"
	)

	IF( CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
		EXECUTE_PROCESS( COMMAND ./b2 toolset=gcc install --prefix=${BOOST_INSTALL_DIR}
			WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE build_result )
	ELSE()
		EXECUTE_PROCESS( COMMAND ./b2 install --prefix=${BOOST_INSTALL_DIR}
			WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE build_result )
	ENDIF()
ELSE( WIN32 )
	EXECUTE_PROCESS( COMMAND ./b2 install
		WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE build_result )
ENDIF( WIN32 )

RETURN( ${build_result} )
