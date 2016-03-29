
SET( GRAPHVIZ_CONFIGURE_COMMAND sh ./configure )

SET( GRAPHVIZ_OPTIONS
	CFLAGS=${GRAPHVIZ_C_FLAGS}
	CXXFLAGS=${GRAPHVIZ_CXX_FLAGS}
	LDFLAGS=${GRAPHVIZ_EXE_LINKER_FLAGS}
	PKG_CONFIG_PATH=${FONTCONFIG_LIBRARY_DIR}/pkgconfig
	--prefix=${INSTALL_DIR}
	--enable-dependency-tracking
	# GraphViz Options
	--without-gdk-pixbuf
	--with-mylibgd
	--disable-swig
	--without-x
	--disable-tcl
	--without-ipsepcola
	--enable-ltdl
	--without-gtk
	--without-gtkgl
	--with-expat=yes
	--with-expatincludedir=${EXPAT_INCLUDE_DIR}
	--with-expatlibdir=${EXPAT_LIBRARY_DIR}
	--with-zincludedir=${ZLIB_INCLUDE_DIR}
	--with-zlibdir=${ZLIB_LIBRARY_DIR}
	--with-fontconfig=yes
	--with-freetype2=yes
	--with-freetype2includedir=${FREETYPE_INCLUDE_DIR}
	--with-freetype2libdir=${FREETYPE_LIBRARY_DIR}
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
