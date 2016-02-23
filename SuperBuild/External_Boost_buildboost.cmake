
SET( BOOST_BUILD_COMMAND
	./b2
)

SET( BOOST_OPTIONS
	variant=debug,release			# debug,release		: build type
	link=static						# shared,static 	: link type
	threading=multi					# single,multi		: thread safety
	address-model=64				# 32,64				: bit type
	runtime-link=static				# shared,static		: version of c/c++ runtimes
	--without-test					# disable building test
	--build-dir=${BUILD_DIR}		# location of boost build files
	--prefix=${BOOST_INSTALL_DIR}	# location of boost installation
)

IF( WIN32 )
	IF( CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
		LIST( APPEND BOOST_OPTIONS
			toolset=gcc
		)
	ENDIF()
ENDIF()

EXECUTE_PROCESS( COMMAND ${BOOST_BUILD_COMMAND} ${BOOST_OPTIONS} install
		WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE build_result )

RETURN( ${build_result} )
