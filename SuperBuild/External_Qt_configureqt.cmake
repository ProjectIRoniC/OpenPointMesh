
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
	-system-libjpeg
	-system-libtiff
	-system-libpng
	-system-libmng
	
	# Include Directories
	-I ${ZLIB_INCLUDE_DIR}
	-I ${JPEG_INCLUDE_DIR}
	-I ${TIFF_INCLUDE_DIR}
	-I ${PNG_INCLUDE_DIR}
	-I ${MNG_INCLUDE_DIR}
	
	# Library Directories
	-L ${ZLIB_LIBRARY_DIR}
	-L ${JPEG_LIBRARY_DIR}
	-L ${TIFF_LIBRARY_DIR}
	-L ${PNG_LIBRARY_DIR}
	-L ${MNG_LIBRARY_DIR}
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
