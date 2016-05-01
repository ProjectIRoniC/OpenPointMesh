
SET( BOOST_BUILD_COMMAND ./b2 )

SET( BOOST_C_FLAGS ${EP_NONCMAKE_COMMON_C_FLAGS} )
SET( BOOST_CXX_FLAGS ${EP_NONCMAKE_COMMON_CXX_FLAGS} )

IF( CMAKE_POSITION_INDEPENDENT_CODE )
	IF( NOT "${BOOST_CXX_FLAGS}" MATCHES "-fPIC" )
		SET( BOOST_CXX_FLAGS "${BOOST_CXX_FLAGS} -fPIC" )
	ENDIF()
	IF( NOT "${BOOST_C_FLAGS}" MATCHES "-fPIC" )
		SET( BOOST_C_FLAGS "${BOOST_C_FLAGS} -fPIC" )
	ENDIF()
ENDIF()

SET( BOOST_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L${BZIP2_LIBRARY_DIR} -L${ZLIB_LIBRARY_DIR}" )

SET( BOOST_OPTIONS
	threading=multi					# single,multi		: thread safety
	address-model=64				# 32,64				: bit type
	--abbreviate-paths				# Compresses target paths by abbreviating each component
	--without-test					# disable building test
	--build-dir=${BUILD_DIR}		# location of boost build files
	--prefix=${BOOST_INSTALL_DIR}	# location of boost installation
	include=${BZIP2_INCLUDE_DIR}	# add bzip2 dependency include directory
	include=${ZLIB_INCLUDE_DIR}		# add zlib dependency include directory
	include=${PYTHON_INCLUDE_DIRS}	# add python dependency include directory
	cxxflags=${BOOST_CXX_FLAGS}		# add cxx flags
	cflags=${BOOST_C_FLAGS}			# add c flags
	linkflags=${BOOST_LINKER_FLAGS}	# add library directories to the linker
)

IF( NOT PYTHONLIBS_FOUND )
	LIST( APPEND BOOST_OPTIONS --without-python )
ENDIF()

IF( "${CMAKE_BUILD_TYPE}" MATCHES "Release" OR "${CMAKE_BUILD_TYPE}" MATCHES "MinSizeRel" )
	LIST( APPEND BOOST_OPTIONS variant=release )	# build type release
ELSE()
	LIST( APPEND BOOST_OPTIONS variant=debug )		# build type debug
ENDIF()

IF( BUILD_SHARED_LIBS )
	LIST( APPEND BOOST_OPTIONS
		link=shared			# link type shared
		runtime-link=shared	# c/c++ runtimes shared
	)
ELSE()
	LIST( APPEND BOOST_OPTIONS
		link=static			# link type static
		runtime-link=static	# c/c++ runtimes static
	)
ENDIF()

IF( WIN32 )
	IF( CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
		LIST( APPEND BOOST_OPTIONS toolset=gcc )	# specify gnu instead of the msvc default
	ENDIF()
ENDIF()

EXECUTE_PROCESS( COMMAND ${BOOST_BUILD_COMMAND} ${BOOST_OPTIONS} install
					WORKING_DIRECTORY ${SOURCE_DIR} RESULT_VARIABLE b2_result )

IF( NOT "${b2_result}" STREQUAL "0" )
	MESSAGE( STATUS "Boost b2 Failed!!!" )
	MESSAGE( FATAL_ERROR "b2_result='${b2_result}'" )
ENDIF()
		
RETURN( ${b2_result} )
