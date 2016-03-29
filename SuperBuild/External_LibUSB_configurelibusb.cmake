
SET( LIBUSB_CONFIGURE_COMMAND sh ./configure )

SET( LIBUSB_OPTIONS
	CFLAGS=${LIBUSB_C_FLAGS}
	LDFLAGS=${LIBUSB_EXE_LINKER_FLAGS}
	--prefix=${INSTALL_DIR}
)

IF( CMAKE_POSITION_INDEPENDENT_CODE )
	LIST( APPEND LIBUSB_OPTIONS --with-pic )	# produce position independent code
ENDIF()

IF( BUILD_SHARED_LIBS )
	LIST( APPEND LIBUSB_OPTIONS
		--enable-shared		# enable shared libraries
		--disable-static	# disable static libraries
	)
ELSE()
	LIST( APPEND LIBUSB_OPTIONS
		--disable-shared	# disable shared libraries
		--enable-static		# enable static libraries
	)
ENDIF()

EXECUTE_PROCESS( COMMAND ${LIBUSB_CONFIGURE_COMMAND} ${LIBUSB_OPTIONS}
			WORKING_DIRECTORY ${SOURCE_DIR} RESULT_VARIABLE configure_result )

IF( NOT "${configure_result}" STREQUAL "0" )
	MESSAGE( STATUS "libusb Configure Failed!!!" )
	MESSAGE( FATAL_ERROR "configure_result='${configure_result}'" )
ENDIF()
			
RETURN( ${configure_result} )
