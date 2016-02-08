#
# Follow the boost suggestions
#
SET( BOOST_OPTIONS
	-q
	"-j 1"					# number of commands to run parallel
	variant=debug,release	# debug,release		: build type
	link=static				# shared,static 	: link type
	threading=multi			# single,multi		: thread safety
	address-model=64		# 32,64				: bit type
	runtime-link=static		# shared,static		: version of c/c++ runtimes
)


IF( WIN32 )
	IF( CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
		EXECUTE_PROCESS( COMMAND ./b2 ${BOOST_OPTIONS} toolset=gcc install --prefix=${BOOST_INSTALL_DIR}
			WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE build_result )
	ELSE()
		EXECUTE_PROCESS( COMMAND ./b2 ${BOOST_OPTIONS} install --prefix=${BOOST_INSTALL_DIR}
			WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE build_result )
	ENDIF()
ELSE( WIN32 )
	EXECUTE_PROCESS( COMMAND ./b2 ${BOOST_OPTIONS} install
		WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE build_result )
ENDIF( WIN32 )

RETURN( ${build_result} )
