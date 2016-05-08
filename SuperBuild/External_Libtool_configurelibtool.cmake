
SET( LIBTOOL_CONFIGURE_COMMAND sh ./configure )

SET( LIBTOOL_OPTIONS
	CFLAGS=${EP_NONCMAKE_COMMON_C_FLAGS}
	LDFLAGS=${CMAKE_EXE_LINKER_C_FLAGS}
	--prefix=${INSTALL_DIR}
)

IF( CMAKE_POSITION_INDEPENDENT_CODE )
	LIST( APPEND LIBTOOL_OPTIONS --with-pic )	# produce position independent code
ENDIF()

IF( BUILD_SHARED_LIBS )
	LIST( APPEND LIBTOOL_OPTIONS
		--enable-shared		# enable shared libraries
		--disable-static	# disable static libraries
	)
ELSE()
	LIST( APPEND LIBTOOL_OPTIONS
		--disable-shared	# disable shared libraries
		--enable-static		# enable static libraries
	)
ENDIF()

EXECUTE_PROCESS( COMMAND ${LIBTOOL_CONFIGURE_COMMAND} ${LIBTOOL_OPTIONS} 
					WORKING_DIRECTORY ${SOURCE_DIR} RESULT_VARIABLE configure_result )

IF( NOT "${configure_result}" STREQUAL "0" )
	MESSAGE( STATUS "Libtool Configure Failed!!!" )
	MESSAGE( FATAL_ERROR "configure_result='${configure_result}'" )
ENDIF()
			
RETURN( ${configure_result} )
