 
SET( AUTORECONF_COMMAND bash -l -c "cd ${AUTORECONF_IN_DIR} && autoreconf -ivf" )

EXECUTE_PROCESS( COMMAND ${AUTORECONF_COMMAND} 
			WORKING_DIRECTORY ${AUTORECONF_IN_DIR} RESULT_VARIABLE autoreconf_result )

IF( NOT "${autoreconf_result}" STREQUAL "0" )
	MESSAGE( STATUS "autoreconf Failed!!!" )
	MESSAGE( FATAL_ERROR "autoreconf_result='${autoreconf_result}'" )
ENDIF()

RETURN( ${autoreconf_result} )
