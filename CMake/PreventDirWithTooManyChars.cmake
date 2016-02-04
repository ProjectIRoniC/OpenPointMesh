#
# This function will prevent building the project from a source or build directory having
# too many characters.
#
# The maximum path length should be specified setting the variable ${PROJECT_NAME}_ROOT_DIR_MAX_LENGTH.
# For example:
#
#   set(${PROJECT_NAME}_ROOT_DIR_MAX_LENGTH 40)
#
# To skip the directory length check, the project can be configured with:
#
#   -D<PROJECT_NAME>_SKIP_ROOT_DIR_MAX_LENGTH_CHECK:BOOL=TRUE
#
FUNCTION( AssureLengthForSourceOrBuildDir max_length )

	FUNCTION( _check_path_length path description )
		STRING( LENGTH "${path}" n )
		IF( n GREATER ${max_length} )
			STRING( SUBSTRING "${path}" 0 ${max_length} _expected_path )
			MESSAGE( FATAL_ERROR
				"The current ${description} directory has too many characters:\n"
				"  current path:\n"
				"    ${path} [${n} chars]\n"
				"  expected path:\n"
				"    ${_expected_path} [${max_length} chars]\n"
				""
				"Please use a shorter directory for ${PROJECT_NAME} ${description} directory.\n"
				""
				"To ignore this error, reconfigure with the following option:\n"
				"  -D${PROJECT_NAME}_SKIP_ROOT_DIR_MAX_LENGTH_CHECK:BOOL=TRUE\n"
			)
		ENDIF()
	ENDFUNCTION()

	_check_path_length( "${CMAKE_CURRENT_SOURCE_DIR}" "source" )
	_check_path_length( "${CMAKE_CURRENT_BINARY_DIR}" "binary" )

ENDFUNCTION()

IF( NOT DEFINED ${PROJECT_NAME}_SKIP_ROOT_DIR_MAX_LENGTH_CHECK )
	SET( ${PROJECT_NAME}_SKIP_ROOT_DIR_MAX_LENGTH_CHECK FALSE )
ENDIF()

IF( NOT ${PROJECT_NAME}_SKIP_ROOT_DIR_MAX_LENGTH_CHECK )
	IF( "${${PROJECT_NAME}_ROOT_DIR_MAX_LENGTH}" STREQUAL "" )
		MESSAGE( FATAL_ERROR "Variable ${PROJECT_NAME}_ROOT_DIR_MAX_LENGTH should be set to an integer value > 0." )
	ENDIF()
	AssureLengthForSourceOrBuildDir( ${${PROJECT_NAME}_ROOT_DIR_MAX_LENGTH} )
ENDIF()

