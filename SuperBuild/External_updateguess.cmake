
# A patch to download the latest version of config.guess
FILE( DOWNLOAD 
	"http://git.savannah.gnu.org/gitweb/?p=config.git;a=blob_plain;f=config.guess;hb=HEAD"
	"${UPDATE_GUESS_IN_DIR}/config.guess"
	SHOW_PROGRESS
	STATUS config_guess_download_result
)

LIST( GET config_guess_download_result 0 guess_result )
IF( NOT "${guess_result}" STREQUAL "0" )
	MESSAGE( STATUS "Update config.guess Failed!!!" )
	MESSAGE( FATAL_ERROR "config_guess_download_result='${config_guess_download_result}'" )
ENDIF()

# A patch to download the latest version of config.sub
FILE( DOWNLOAD 
	"http://git.savannah.gnu.org/gitweb/?p=config.git;a=blob_plain;f=config.sub;hb=HEAD"
	"${UPDATE_GUESS_IN_DIR}/config.sub"
	SHOW_PROGRESS
	STATUS config_sub_download_result
)

LIST( GET config_sub_download_result 0 sub_result )
IF( NOT "${sub_result}" STREQUAL "0" )
	MESSAGE( STATUS "Update config.sub Failed!!!" )
	MESSAGE( FATAL_ERROR "config_sub_download_result='${config_sub_download_result}'" )
ENDIF()
			
RETURN( ${sub_result} )
