
SET( AUTOGENSH_COMMAND bash -l -c "cd ${AUTOGENSH_IN_DIR} && sh ./autogen.sh" )

EXECUTE_PROCESS( COMMAND ${AUTOGENSH_COMMAND} 
			WORKING_DIRECTORY ${AUTOGENSH_IN_DIR} RESULT_VARIABLE autogensh_result )

IF( NOT "${autogensh_result}" STREQUAL "0" )
	MESSAGE( STATUS "autogen.sh Failed!!!" )
	MESSAGE( FATAL_ERROR "autogensh_result='${autogensh_result}'" )
ENDIF()

RETURN( ${autogensh_result} )
