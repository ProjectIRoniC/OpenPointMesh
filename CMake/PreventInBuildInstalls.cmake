# Adapated from ITKv4/CMake/PreventInBuildInstalls.cmake
STRING( TOLOWER "${CMAKE_INSTALL_PREFIX}" _PREFIX )
STRING( TOLOWER "${${CMAKE_PROJECT_NAME}_BINARY_DIR}" _BUILD )
IF( "${_PREFIX}" STREQUAL "${_BUILD}" )
	MESSAGE( FATAL_ERROR
		"The current CMAKE_INSTALL_PREFIX points at the build tree:\n"
		"  ${CMAKE_INSTALL_PREFIX}\n"
		"This is not supported."
	)
ENDIF()
