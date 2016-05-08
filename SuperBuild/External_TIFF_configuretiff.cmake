
SET( TIFF_CONFIGURE_COMMAND sh ./configure )

SET( TIFF_OPTIONS
	CFLAGS=${EP_NONCMAKE_COMMON_C_FLAGS}
	CXXFLAGS=${EP_NONCMAKE_COMMON_CXX_FLAGS}
	LDFLAGS=${CMAKE_EXE_LINKER_FLAGS}
	--prefix=${INSTALL_DIR}
	--enable-dependency-tracking
	--enable-cxx
	# ZLIB ARGS
	--with-zlib
	--with-zlib-include-dir=${ZLIB_INCLUDE_DIR}
	--with-zlib-lib-dir=${ZLIB_LIBRARY_DIR}
	# JPEG ARGS
	--enable-jpeg
	--with-jpeg-include-dir=${JPEG_INCLUDE_DIR}
	--with-jpeg-lib-dir=${JPEG_LIBRARY_DIR}
)

IF( CMAKE_POSITION_INDEPENDENT_CODE )
	LIST( APPEND TIFF_OPTIONS --with-pic )	# produce position independent code
ENDIF()

IF( BUILD_SHARED_LIBS )
	LIST( APPEND TIFF_OPTIONS
		--enable-shared		# enable shared libraries
		--disable-static	# disable static libraries
	)
ELSE()
	LIST( APPEND TIFF_OPTIONS
		--disable-shared	# disable shared libraries
		--enable-static		# enable static libraries
	)
ENDIF()

EXECUTE_PROCESS( COMMAND ${TIFF_CONFIGURE_COMMAND} ${TIFF_OPTIONS} 
					WORKING_DIRECTORY ${SOURCE_DIR} RESULT_VARIABLE configure_result )

IF( NOT "${configure_result}" STREQUAL "0" )
	MESSAGE( STATUS "TIFF Configure Failed!!!" )
	MESSAGE( FATAL_ERROR "configure_result='${configure_result}'" )
ENDIF()
			
RETURN( ${configure_result} )
