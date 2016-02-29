
SET( GRAPHVIZ_CONFIGURE_COMMAND sh ./configure )

SET( GRAPHVIZ_OPTIONS
	CFLAGS=${GRAPHVIZ_C_FLAGS}
	CXXFLAGS=${GRAPHVIZ_CXX_FLAGS}
	--with-zlib
	--with-zlib-include-dir=${ZLIB_INCLUDE_DIR}
	--with-zlib-lib-dir=${ZLIB_LIBRARY_DIR}
	--with-png
	--with-png-include-dir=${PNG_INCLUDE_DIR}
	--with-png-lib-dir=${PNG_LIBRARY_DIR}
	--prefix=${INSTALL_DIR}
)

IF( CMAKE_POSITION_INDEPENDENT_CODE )
	LIST( APPEND GRAPHVIZ_OPTIONS --with-pic )	# produce position independent code
ENDIF()

IF( BUILD_SHARED_LIBS )
	LIST( APPEND GRAPHVIZ_OPTIONS
		--enable-shared		# enable shared libraries
		--disable-static	# disable static libraries
	)
ELSE()
	LIST( APPEND GRAPHVIZ_OPTIONS
		--disable-shared	# disable shared libraries
		--enable-static		# enable static libraries
	)
ENDIF()

EXECUTE_PROCESS( COMMAND ${GRAPHVIZ_CONFIGURE_COMMAND} ${GRAPHVIZ_OPTIONS} 
			WORKING_DIRECTORY ${SOURCE_DIR} RESULT_VARIABLE configure_result )

IF( NOT "${configure_result}" STREQUAL "0" )
	MESSAGE( STATUS "GraphViz Configure Failed!!!" )
	MESSAGE( FATAL_ERROR "configure_result='${configure_result}'" )
ENDIF()
			
RETURN( ${configure_result} )
