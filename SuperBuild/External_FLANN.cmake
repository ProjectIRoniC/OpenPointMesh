# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced
SET( extProjName FLANN ) #The find_package known name
SET( proj        FLANN ) #This local name
SET( ${extProjName}_REQUIRED_VERSION "1.8.4" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES "" )

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

IF( NOT ( DEFINED "USE_SYSTEM_${extProjName}" AND "${USE_SYSTEM_${extProjName}}" ) )
	### --- Project specific additions here
	SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
	SET( ${proj}_CMAKE_OPTIONS
		-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
		-DCMAKE_CXX_FLAGS:STRING=${ep_common_cxx_flags}
		-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
		-DCMAKE_C_FLAGS:STRING=${ep_common_c_flags}
		-DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
		-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
		-DBUILD_MATLAB_BINDINGS:BOOL=OFF
		-DBUILD_EXAMPLES:BOOL=OFF
		-DBUILD_TESTS:BOOL=OFF
	)
	
	SET( ${proj}_REPOSITORY "${git_protocol}://github.com/mariusmuja/flann.git" )
	SET( ${proj}_GIT_TAG master )  # master 
	### --- End Project specific additions

	ExternalProject_Add( ${proj}
		${${proj}_EP_ARGS}
		GIT_REPOSITORY ${${proj}_REPOSITORY}
		GIT_TAG ${${proj}_GIT_TAG}
		SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj}
		BINARY_DIR ${proj}-build
		LOG_CONFIGURE 0  # Wrap configure in script to ignore log output from dashboards
		LOG_BUILD     0  # Wrap build in script to to ignore log output from dashboards
		LOG_TEST      0  # Wrap test in script to to ignore log output from dashboards
		LOG_INSTALL   0  # Wrap install in script to to ignore log output from dashboards
		${cmakeversion_external_update} "${cmakeversion_external_update_value}"
		INSTALL_DIR ${${proj}_INSTALL_DIR}
		CMAKE_GENERATOR ${gen}
		CMAKE_ARGS -Wno-dev --no-warn-unused-cli
		CMAKE_CACHE_ARGS ${${proj}_CMAKE_OPTIONS}
		## We really do want to install in order to limit # of include paths INSTALL_COMMAND ""
		DEPENDS
			${${proj}_DEPENDENCIES}
	)
	
	### --- Set binary information
	SET( ${extProjName}_DIR ${CMAKE_BINARY_DIR}/${proj}-install )
	SET( ${extProjName}_INCLUDE_DIR ${CMAKE_BINARY_DIR}/${proj}-install/include/flann )
	### --- End binary information
ELSE()
	IF( ${USE_SYSTEM_${extProjName}} )
		FIND_PACKAGE( ${extProjName} ${${extProjName}_REQUIRED_VERSION} REQUIRED )
		MESSAGE( STATUS "USING the system ${extProjName}, set ${extProjName}_DIR=${${extProjName}_DIR}" )
	ENDIF()
	# The project is provided using ${extProjName}_DIR, nevertheless since other
	# project may depend on ${extProjName}, let's add an 'empty' one
	ExternalProject_Add_Empty( ${proj} "${${proj}_DEPENDENCIES}" )
ENDIF()

mark_as_superbuild(
	VARS
		${extProjName}_DIR:PATH
		${extProjName}_INCLUDE_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "FLANN_DIR:${${extProjName}_DIR}" )
ExternalProject_Message( ${proj} "FLANN_INCLUDE_DIR:${${extProjName}_INCLUDE_DIR}" )
