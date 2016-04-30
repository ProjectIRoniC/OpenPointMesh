
SET( GENERATE_ALIAS_COMMAND make )

# fc-case
MESSAGE( STATUS "fc-case" )
SET( FC_CASE_ALIAS_SOURCE_DIR ${SOURCE_DIR}/fc-case )
SET( FC_CASE_ALIAS_MAKE_TARGETS
		fcalias.h
		fcaliastail.h
		fccase.h
		fc-case
)

EXECUTE_PROCESS( COMMAND ${GENERATE_ALIAS_COMMAND} ${FC_CASE_ALIAS_MAKE_TARGETS}
					WORKING_DIRECTORY ${FC_CASE_ALIAS_SOURCE_DIR} RESULT_VARIABLE fc_case_configure_result )

IF( NOT "${fc_case_configure_result}" STREQUAL "0" )
	MESSAGE( STATUS "FontConfig Configure Failed!!!" )
	MESSAGE( FATAL_ERROR "fc_case_configure_result='${fc_case_configure_result}'" )
ENDIF()

# fc-lang
MESSAGE( STATUS "fc-lang" )
SET( FC_LANG_ALIAS_SOURCE_DIR ${SOURCE_DIR}/fc-lang )
SET( FC_LANG_ALIAS_MAKE_TARGETS
		fcalias.h
		fcaliastail.h
		fclang.h
		fc-lang
)

EXECUTE_PROCESS( COMMAND ${GENERATE_ALIAS_COMMAND} ${FC_LANG_ALIAS_MAKE_TARGETS}
					WORKING_DIRECTORY ${FC_LANG_ALIAS_SOURCE_DIR} RESULT_VARIABLE fc_lang_configure_result )

IF( NOT "${fc_lang_configure_result}" STREQUAL "0" )
	MESSAGE( STATUS "FontConfig Configure Failed!!!" )
	MESSAGE( FATAL_ERROR "fc_lang_configure_result='${fc_lang_configure_result}'" )
ENDIF()

# fc-glyphname
MESSAGE( STATUS "fc-glyphname" )
SET( FC_GLYPHNAME_ALIAS_SOURCE_DIR ${SOURCE_DIR}/fc-glyphname )
SET( FC_GLYPHNAME_ALIAS_MAKE_TARGETS
		fcalias.h
		fcaliastail.h
		fcglyphname.h
		fc-glyphname
)

EXECUTE_PROCESS( COMMAND ${GENERATE_ALIAS_COMMAND} ${FC_GLYPHNAME_ALIAS_MAKE_TARGETS}
					WORKING_DIRECTORY ${FC_GLYPHNAME_ALIAS_SOURCE_DIR} RESULT_VARIABLE fc_glyphname_configure_result )

IF( NOT "${fc_glyphname_configure_result}" STREQUAL "0" )
	MESSAGE( STATUS "FontConfig Configure Failed!!!" )
	MESSAGE( FATAL_ERROR "fc_lang_configure_result='${fc_glyphname_configure_result}'" )
ENDIF()

# fc src
MESSAGE( STATUS "src" )
SET( FC_SRC_ALIAS_SOURCE_DIR ${SOURCE_DIR}/src )
SET( FC_SRC_ALIAS_MAKE_TARGETS
		fcalias.h
		stamp-fcstdint
		fcobjshash.gperf
		fcobjshash.h
		fcftalias.h
		fcftaliastail.h
)


EXECUTE_PROCESS( COMMAND ${GENERATE_ALIAS_COMMAND} ${FC_SRC_ALIAS_MAKE_TARGETS}
					WORKING_DIRECTORY ${FC_SRC_ALIAS_SOURCE_DIR} RESULT_VARIABLE fc_src_configure_result )

IF( NOT "${fc_src_configure_result}" STREQUAL "0" )
	MESSAGE( STATUS "FontConfig Configure Failed!!!" )
	MESSAGE( FATAL_ERROR "fc_src_configure_result='${fc_src_configure_result}'" )
ENDIF()
			
RETURN( ${fc_src_configure_result} )
