#
# This function will prevent building the project from a source or build directory having spaces
#
# To allow directory with spaces, the project can be configured with:
#
#   -D<PROJECT_NAME>_SKIP_DIR_WITH_SPACES_CHECK:BOOL=TRUE
#
FUNCTION( AssureNoSpacesForSourceOrBuildDir )

	FUNCTION( _check_path path description )

		# make sure the user doesn't play dirty with symlinks
		GET_FILENAME_COMPONENT( real_path "${path}" REALPATH )

		STRING( FIND ${path} " " real_path_space_position )
		SET( real_path_contains_spaces FALSE )
		IF( NOT real_path_space_position EQUAL -1 )
			SET( real_path_contains_spaces TRUE )
		ENDIF()

		IF( real_path_contains_spaces )
			MESSAGE( FATAL_ERROR
				"The current ${description} directory contains spaces:\n"
				"  ${real_path}\n"
				"Building the project from a source or build directory having spaces is NOT recommended or tested !\n"
				""
				"To ignore this error, reconfigure with the following option:\n"
				"  -D${PROJECT_NAME}_SKIP_DIR_WITH_SPACES_CHECK:BOOL=TRUE\n"
			)
		ENDIF()

	ENDFUNCTION()

	_check_path( "${CMAKE_SOURCE_DIR}" "source" )
	_check_path( "${CMAKE_BINARY_DIR}" "binary" )

ENDFUNCTION()

IF( NOT DEFINED ${PROJECT_NAME}_SKIP_DIR_WITH_SPACES_CHECK )
	SET( ${PROJECT_NAME}_SKIP_DIR_WITH_SPACES_CHECK FALSE )
ENDIF()

IF( NOT ${PROJECT_NAME}_SKIP_DIR_WITH_SPACES_CHECK )
	AssureNoSpacesForSourceOrBuildDir()
ENDIF()
