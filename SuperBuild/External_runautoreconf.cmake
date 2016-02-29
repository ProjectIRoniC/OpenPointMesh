SET( ENV{MSYSTEM} "MINGW64" )
SET( ENV{PATH} "c:/msys64/usr/share/bash-completion/completions$ENV{PATH}" ) 
SET( AUTORECONF_COMMAND bash -l -c "autoreconf -ivvf" )
SET( COMMAND_1 bash -l  )
SET( COMMAND_2 bash autoreconf -ivf )
SET( AUTORECONF_COMMAND_T echo $ENV{PATH} )
SET( AUTORECONF_ARGS -ivf )


EXECUTE_PROCESS( #COMMAND ${COMMAND_1} 
					COMMAND ${COMMAND_2}
			WORKING_DIRECTORY ${AUTORECONF_IN_DIR} RESULT_VARIABLE autoreconf_result )

IF( NOT "${autoreconf_result}" STREQUAL "0" )
	MESSAGE( STATUS "autoreconf Failed!!!" )
	MESSAGE( FATAL_ERROR "autoreconf_result='${autoreconf_result}'" )
ENDIF()
MESSAGE( FATAL_ERROR stop )
RETURN( ${autoreconf_result} )
