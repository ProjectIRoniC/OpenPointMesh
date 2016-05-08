# Adapated from ITKv4/CMake/PreventInSourceBuilds.cmake
#
# This function will prevent in-source builds
FUNCTION( AssureOutOfSourceBuilds PROJECT_NAME )
	# make sure the user doesn't play dirty with symlinks
	GET_FILENAME_COMPONENT( srcdir "${CMAKE_SOURCE_DIR}" REALPATH )
	GET_FILENAME_COMPONENT( bindir "${CMAKE_BINARY_DIR}" REALPATH )

	# disallow in-source builds
	IF( "${srcdir}" STREQUAL "${bindir}" )
		MESSAGE( "###########################################################################" )
		MESSAGE( "# ${PROJECT_NAME} should not be configured & built in the ${PROJECT_NAME} source directory" )
		MESSAGE( "# You must run cmake in a build directory." )
		MESSAGE( "# For example:" )
		MESSAGE( "#" )
		MESSAGE( "#     mkdir ${PROJECT_NAME}-Sandbox ; cd ${PROJECT_NAME}-sandbox" )
		MESSAGE( "#" )
		MESSAGE( "# Check out source code in ${PROJECT_NAME}-sandbox" )
		MESSAGE( "#" )
		MESSAGE( "#     mkdir ${PROJECT_NAME}-build" )
		MESSAGE( "#" )
		MESSAGE( "# this will create the following directory structure" )
		MESSAGE( "#" )
		MESSAGE( "# ${PROJECT_NAME}-Sandbox" )
		MESSAGE( "#  +--${PROJECT_NAME}" )
		MESSAGE( "#  +--${PROJECT_NAME}-build" )
		MESSAGE( "#" )
		MESSAGE( "# Then you can proceed to configure and build" )
		MESSAGE( "# by using the following commands" )
		MESSAGE( "#" )
		MESSAGE( "#     cd ${PROJECT_NAME}-build" )
		MESSAGE( "#     cmake ../${PROJECT_NAME} # or ccmake, or cmake-gui" )
		MESSAGE( "#     make" )
		MESSAGE( "#" )
		MESSAGE( "# NOTE: Given that you already tried to make an in-source build" )
		MESSAGE( "#       CMake have already created several files & directories" )
		MESSAGE( "#       in your source tree." )
		MESSAGE( "#" )
		MESSAGE( "# The following command will show you all files not part of ${PROJECT_NAME}" )
		MESSAGE( "#       cd ${PROJECT_NAME}-Sandbox/${PROJECT_NAME}" )
		MESSAGE( "#       svn status | grep '[^?]' | awk '{print \$2}'" )
		MESSAGE( "#" )
		MESSAGE( "# WARNING: if you have added files to ${PROJECT_NAME} but not used svn add" )
		MESSAGE( "# to add them to SVN's version control, this command will display them" )
		MESSAGE( "# along with the files CMake created during configuration.  You will need" )
		MESSAGE( "# to either save them outside the ${PROJECT_NAME} source tree, or run svn add" )
		MESSAGE( "# to let SVN know they are legitimate source files." )
		MESSAGE( "# Once you've verified that all unknown files are the result of CMake" )
		MESSAGE( "# configuration, you can run this command to clean them up" )
		MESSAGE( "#       svn status | grep '[^?]' | awk '{print \$2}' | xargs rm -fr" )
		MESSAGE( "###########################################################################" )
		MESSAGE( FATAL_ERROR "Quitting configuration" )
	ENDIF()
ENDFUNCTION()

AssureOutOfSourceBuilds( ${CMAKE_PROJECT_NAME} )
