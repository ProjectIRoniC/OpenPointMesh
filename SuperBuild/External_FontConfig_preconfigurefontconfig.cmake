
SET( FONTCONFIG_FREETYPE_LIBS_ARG "-L${FREETYPE_LIBRARY_DIR} -l${FREETYPE_LIBRARY_NAME} -L${PNG_LIBRARY_DIR} -l${PNG_LIBRARY_NAME} -L${BZIP2_LIBRARY_DIR} -l${BZIP2_LIBRARY_NAME} -L${ZLIB_LIBRARY_DIR} -l${ZLIB_LIBRARY_NAME}" )
SET( FONTCONFIG_FREETYPE_CFLAGS_ARG "-I${FREETYPE_INCLUDE_DIR}" )

SET( FONTCONFIG_OPTIONS
	CFLAGS=\"${EP_NONCMAKE_COMMON_C_FLAGS}\"
	LDFLAGS=\"${CMAKE_EXE_LINKER_C_FLAGS}\"
	FREETYPE_LIBS=\"${FONTCONFIG_FREETYPE_LIBS_ARG}\"
	FREETYPE_CFLAGS=\"${FONTCONFIG_FREETYPE_CFLAGS_ARG}\"
	--with-expat=${EXPAT_DIR}
	--with-expat-includes=${EXPAT_INCLUDE_DIR}
	--with-expat-lib=${EXPAT_LIBRARY_DIR}
	--prefix=${INSTALL_DIR}
	--enable-dependency-tracking
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

STRING( REPLACE ";" " " AUTOGEN_OPTIONS_STRING "${FONTCONFIG_OPTIONS}" )
EXECUTE_PROCESS( COMMAND ${CMAKE_COMMAND}
			# Autogen ARGS
			-DAUTOGENSH_IN_DIR:PATH=${SOURCE_DIR}
			-DAUTOGEN_OPTIONS:STRING=${AUTOGEN_OPTIONS_STRING}
			# Use the configure script
			-P ${RUN_AUTOGENSH_SCRIPT}
			RESULT_VARIABLE configure_result
)

IF( NOT "${configure_result}" STREQUAL "0" )
	MESSAGE( STATUS "run_autogen_sh Failed!!!" )
	MESSAGE( FATAL_ERROR "configure_result='${configure_result}'" )
ENDIF()
			
RETURN( ${configure_result} )
