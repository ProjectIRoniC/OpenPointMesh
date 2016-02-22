
SET( BOOST_BUILD_COMMAND
	./b2
)

SET( BOOST_OPTIONS
	-q
	-j 5					# number of commands to run parallel
	variant=debug,release	# debug,release		: build type
	link=static				# shared,static 	: link type
	threading=multi			# single,multi		: thread safety
	address-model=64		# 32,64				: bit type
	runtime-link=static		# shared,static		: version of c/c++ runtimes
	--without-test			# disable building test
)

IF( WIN32 )
	IF( CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
		LIST( APPEND BOOST_OPTIONS
			toolset=gcc
		)
	ENDIF()
ENDIF()

EXECUTE_PROCESS( COMMAND ${BOOST_BUILD_COMMAND} ${BOOST_OPTIONS} install --prefix=${BOOST_INSTALL_DIR}
		WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE build_result )

RETURN( ${build_result} )
