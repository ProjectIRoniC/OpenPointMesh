
SET( FONTCONFIG_CONFIGURE_COMMAND sh ./configure )

SET( FONTCONFIG_OPTIONS
	CFLAGS=${FONTCONFIG_C_FLAGS}
	CXXFLAGS=${FONTCONFIG_CXX_FLAGS}
	LDFLAGS=${FONTCONFIG_EXE_LINKER_FLAGS}
	--prefix=${INSTALL_DIR}
	--enable-dependency-tracking
	--disable-docs
)

IF( CMAKE_POSITION_INDEPENDENT_CODE )
	LIST( APPEND FONTCONFIG_OPTIONS --with-pic )	# produce position independent code
ENDIF()

IF( BUILD_SHARED_LIBS )
	LIST( APPEND FONTCONFIG_OPTIONS
		--enable-shared		# enable shared libraries
		--disable-static	# disable static libraries
	)
ELSE()
	LIST( APPEND FONTCONFIG_OPTIONS
		--disable-shared	# disable shared libraries
		--enable-static		# enable static libraries
	)
ENDIF()

EXECUTE_PROCESS( COMMAND ${FONTCONFIG_CONFIGURE_COMMAND} ${FONTCONFIG_OPTIONS} 
			WORKING_DIRECTORY ${SOURCE_DIR} RESULT_VARIABLE configure_result )

IF( NOT "${configure_result}" STREQUAL "0" )
	MESSAGE( STATUS "FontConfig Configure Failed!!!" )
	MESSAGE( FATAL_ERROR "configure_result='${configure_result}'" )
ENDIF()
			
RETURN( ${configure_result} )
