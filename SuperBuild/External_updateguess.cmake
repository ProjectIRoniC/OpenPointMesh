
# A patch to download the latest version of config.guess
FILE( DOWNLOAD 
	"http://git.savannah.gnu.org/gitweb/?p=config.git;a=blob_plain;f=config.guess;hb=HEAD"
	"${BUILD_DIR}/config.guess"
)

# A patch to download the latest version of config.sub
FILE( DOWNLOAD 
	"http://git.savannah.gnu.org/gitweb/?p=config.git;a=blob_plain;f=config.sub;hb=HEAD"
	"${BUILD_DIR}/config.sub"
)
