
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
	-qt-libmng
	
	# Dependency directories
	-I ${ZLIB_INCLUDE_DIR}
	-I ${JPEG_INCLUDE_DIR}
	-I ${TIFF_INCLUDE_DIR}
	-I ${PNG_INCLUDE_DIR}
	-I ${LCMS_INCLUDE_DIR}
	-I ${MNG_INCLUDE_DIR}
	
	-L ${ZLIB_LIBRARY_DIR}
	-L ${JPEG_LIBRARY_DIR}
	-L ${TIFF_LIBRARY_DIR}
	-L ${PNG_LIBRARY_DIR}
	-L ${LCMS_LIBRARY_DIR}
	-L ${MNG_LIBRARY_DIR}
	
	
	# Libraries to use
	# IMPORTANT:	* libraries must be listed after all include directories
	#				* libraries that support other libraries must be listed first
	#				* all previously linked libraries must be included (ex. LCMS for MNG)
	-l ${ZLIB_LIBRARY}
	-l ${JPEG_LIBRARY}
	-l ${TIFF_LIBRARY}
	-l ${TIFFXX_LIBRARY}
	-l ${PNG_LIBRARY}
	-l ${LCMS_LIBRARY}
	-l ${MNG_LIBRARY}
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
