
SET( TIFF_CONFIGURE_COMMAND
	sh ./configure
)
	
SET( TIFF_OPTIONS
	CFLAGS=${TIFF_C_FLAGS}
	CXXFLAGS=${TIFF_CXX_FLAGS}
	--disable-shared
	--enable-static
	--with-pic
	--with-zlib
	--with-zlib-include-dir=${ZLIB_INCLUDE_DIR}
	--with-zlib-lib-dir=${ZLIB_LIBRARY_DIR}
	--with-jpeg
	--with-jpeg-include-dir=${JPEG_INCLUDE_DIR}
	--with-jpeg-lib-dir=${JPEG_LIBRARY_DIR}
	--prefix=${INSTALL_DIR}
)

EXECUTE_PROCESS( COMMAND ${TIFF_CONFIGURE_COMMAND} ${TIFF_OPTIONS} 
			WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE build_result )

RETURN( ${build_result} )
