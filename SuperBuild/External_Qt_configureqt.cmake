
SET( QT_CONFIGURE_COMMAND ./configure )

SET( QT_OPTIONS
	-opensource			# Compile and link the Open-Source Edition of Qt
	-confirm-license	# Confirm the license selection
	-nomake examples	# Don't compile examples
	-nomake demos		# Don't compile demos
	-nomake doc			# Don't compose documentation
	-no-accessibility	# Do not compile Windows Active Accessibility support
	-no-openvg			# Disables OpenVG functionality
	-no-qt3support		# Disables the Qt 3 support functionality
	-no-ltcg			# Don't use link time code generation
	
	-system-zlib		# Use compiled zlib
	-system-libjpeg		# Use compiled jpeg
	-system-libtiff		# Use compiled tiff
	-system-libpng		# Use compiled png
	-system-libmng		# Use compiled mng
	
	# Include Directories
	
	-I ${JPEG_INCLUDE_DIR}
	-I ${LCMS_INCLUDE_DIR}
	-I ${MNG_INCLUDE_DIR}
	-I ${PNG_INCLUDE_DIR}
	-I ${TIFF_INCLUDE_DIR}
	-I ${ZLIB_INCLUDE_DIR}
	
	# Library Directories
	-L ${JPEG_LIBRARY_DIR}
	-L ${LCMS_LIBRARY_DIR}
	-L ${MNG_LIBRARY_DIR}
	-L ${PNG_LIBRARY_DIR}
	-L ${TIFF_LIBRARY_DIR}
	-L ${ZLIB_LIBRARY_DIR}
	
	# Explicit libraries to include
	-l ${LCMS_LIBRARY_NAME}
)

IF( "${CMAKE_BUILD_TYPE}" MATCHES "Release" OR "${CMAKE_BUILD_TYPE}" MATCHES "MinSizeRel" )
	LIST( APPEND QT_OPTIONS
		-release	# build type release
	)
ELSE()
	LIST( APPEND QT_OPTIONS -debug )	# build type debug
ENDIF()

IF( BUILD_SHARED_LIBS )
	LIST( APPEND QT_OPTIONS
		-shared # build shared libraries
	)
ELSE()
	LIST( APPEND QT_OPTIONS
		-static	# build static libraries
	)
ENDIF()

IF( WIN32 )
	SET( QT_CONFIGURE_COMMAND ./configure.exe )
	
	LIST( APPEND QT_OPTIONS
		-no-dbus		# Do not compile in D-Bus support
		-no-declarative	# Do not build the declarative module
		-no-multimedia	# Do not compile the multimedia module
		-no-phonon		# Do not compile in the Phonon module
		-no-script		# Do not build the QtScript module
		-no-scripttools	# Do not build the QtScriptTools module
		-no-webkit		# Do not compile in the WebKit module
	)
	
	IF( CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU" )
		LIST( APPEND QT_OPTIONS
			-platform win32-g++	# Specify the win32-g++ toolset
			-no-vcproj			# Do not generate VC++ .vcproj files
		)
	ENDIF()
ENDIF()

EXECUTE_PROCESS( COMMAND ${QT_CONFIGURE_COMMAND} ${QT_OPTIONS} 
			WORKING_DIRECTORY ${SOURCE_DIR} RESULT_VARIABLE configure_result )

IF( NOT "${configure_result}" STREQUAL "0" )
	MESSAGE( STATUS "Qt Configure Failed!!!" )
	MESSAGE( FATAL_ERROR "configure_result='${configure_result}'" )
ENDIF()
			
RETURN( ${configure_result} )
