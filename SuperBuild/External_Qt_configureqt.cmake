
SET( QT_OPTIONS
	-debug-and-release
	-opensource
	-confirm-license
	-static
	-ltcg
	-fast
	-nomake examples
	-nomake demos
	-nomake doc
	-no-accessibility
	-no-openvg
	-no-qt3support
	-system-zlib
	-I ${ZLIB_INCLUDE_DIR}
	-L ${ZLIB_LIBRARY_DIR}
	-system-libtiff
	-I ${TIFF_INCLUDE_DIR}
	-L ${TIFF_LIBRARY_DIR}
	-system-libpng
	-I ${PNG_INCLUDE_DIR}
	-L ${PNG_LIBRARY_DIR}
	-system-libmng
	-I ${MNG_INCLUDE_DIR}
	-L ${MNG_LIBRARY_DIR}
	-system-libjpeg
	-I ${JPEG_INCLUDE_DIR}
	-L ${JPEG_LIBRARY_DIR}
)

IF( WIN32 )
	SET( QT_CONFIGURE_COMMAND
		./configure.exe
	)
	
	LIST( APPEND QT_OPTIONS
			-no-dbus
			-no-declarative
			-no-multimedia
			-no-phonon
			-no-script
			-no-scripttools
			-no-webkit
	)
	
	IF( CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
		LIST( APPEND QT_OPTIONS
			-platform win32-g++
			-no-vcproj
		)
	ENDIF()
ELSE()
	SET( QT_CONFIGURE_COMMAND
		./configure
	)
ENDIF()

EXECUTE_PROCESS( COMMAND ${QT_CONFIGURE_COMMAND} ${QT_OPTIONS} 
			WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE build_result )

RETURN( ${build_result} )
