
SET( GRAPHVIZ_CONFIGURE_COMMAND sh ./configure )

SET( GRAPHVIZ_C_FLAGS "${EP_NONCMAKE_COMMON_C_FLAGS} -D_DLL" )
SET( GRAPHVIZ_EXE_FLAGS "-v ${CMAKE_EXE_LINKER_C_FLAGS}" )

# Hack to fix multiple definitions in static builds
# Link Time Optimization might be able to fix this when we get that working
IF( NOT BUILD_SHARED_LIBS )
	SET( GRAPHVIZ_C_FLAGS "${GRAPHVIZ_C_FLAGS} -Wl,--allow-multiple-definition" )
ENDIF()

SET( GRAPHVIZ_FONTCONFIG_CFLAGS "-I${FONTCONFIG_INCLUDE_DIR} -I${EXPAT_INCLUDE_DIR}" )
SET( GRAPHVIZ_FONTCONFIG_LIBS "-L${FONTCONFIG_LIBRARY_DIR} -l${FONTCONFIG_LIBRARY_NAME} -L${EXPAT_LIBRARY_DIR} -l${EXPAT_LIBRARY_NAME}" )

SET( GRAPHVIZ_FREETYPE2_CFLAGS "-I${FREETYPE_INCLUDE_DIR} -I${BZIP2_INCLUDE_DIR} -I${PNG_INCLUDE_DIR}" )
SET( GRAPHVIZ_FREETYPE2_LIBS "-L${FREETYPE_LIBRARY_DIR} -l${FREETYPE_LIBRARY_NAME} -L${BZIP2_LIBRARY_DIR} -l${BZIP2_LIBRARY_NAME} -L${PNG_LIBRARY_DIR} -l${PNG_LIBRARY_NAME}" )

SET( GRAPHVIZ_OPTIONS
	#PKG_CONFIG_PATH=${FONTCONFIG_LIBRARY_DIR}/pkgconfig
	CFLAGS=${GRAPHVIZ_C_FLAGS}
	LDFLAGS=${GRAPHVIZ_EXE_FLAGS}
	FONTCONFIG_CFLAGS=${GRAPHVIZ_FONTCONFIG_CFLAGS}
	FONTCONFIG_LIBS=${GRAPHVIZ_FONTCONFIG_LIBS}
	FREETYPE2_CFLAGS=${GRAPHVIZ_FREETYPE2_CFLAGS}
	FREETYPE2_LIBS=${GRAPHVIZ_FREETYPE2_LIBS}
	--prefix=${INSTALL_DIR}
	--enable-dependency-tracking
	# GraphViz Options
	--without-gdk-pixbuf
	--with-mylibgd
	--disable-swig
	--without-x
	--disable-tcl
	--without-ipsepcola
	--with-ltdl-include=${LIBTOOL_INCLUDE_DIR}
	--with-ltdl-lib=${LIBTOOL_LIBRARY_DIR}
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

IF( WIN32 )
	IF( CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
		LIST( APPEND GRAPHVIZ_OPTIONS --host=x86_64-w64-mingw32 )	# specify the mingw-w64 toolset
	ENDIF()
ENDIF()

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
