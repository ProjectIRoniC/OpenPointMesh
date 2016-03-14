
SET( OPENNI2_CONFIGURE_COMMAND sh ./configure )

SET( OPENNI2_OPTIONS
	CFLAGS=${OPENNI2_C_FLAGS}
	CXXFLAGS=${OPENNI2_CXX_FLAGS}
	--prefix=${INSTALL_DIR}
	--enable-dependency-tracking
)

IF( CMAKE_POSITION_INDEPENDENT_CODE )
	LIST( APPEND OPENNI2_OPTIONS --with-pic )	# produce position independent code
ENDIF()

IF( BUILD_SHARED_LIBS )
	LIST( APPEND OPENNI2_OPTIONS
		--enable-shared		# enable shared libraries
		--disable-static	# disable static libraries
	)
ELSE()
	LIST( APPEND OPENNI2_OPTIONS
		--disable-shared	# disable shared libraries
		--enable-static		# enable static libraries
	)
ENDIF()

EXECUTE_PROCESS( COMMAND ${OPENNI2_CONFIGURE_COMMAND} ${OPENNI2_OPTIONS} 
			WORKING_DIRECTORY ${SOURCE_DIR} RESULT_VARIABLE configure_result )

IF( NOT "${configure_result}" STREQUAL "0" )
	MESSAGE( STATUS "OPENNI2 Configure Failed!!!" )
	MESSAGE( FATAL_ERROR "configure_result='${configure_result}'" )
ENDIF()
			
RETURN( ${configure_result} )
