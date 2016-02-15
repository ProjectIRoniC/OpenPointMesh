# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName Eigen ) # The find_package known name
SET( proj        Eigen ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES "" )

# Include dependent projects if any
ExternalProject_Include_Dependencies(${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES)

### --- Project specific additions here
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_CMAKE_OPTIONS
	-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
	-DCMAKE_CXX_FLAGS:STRING=${ep_common_cxx_flags}
	-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
	-DCMAKE_C_FLAGS:STRING=${ep_common_c_flags}
	-DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
	-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
	-DEIGEN_BUILD_PKGCONFIG:BOOL=OFF
	-DEIGEN_TEST_NOQT:BOOL=ON
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL http://bitbucket.org/eigen/eigen/get/3.2.7.tar.gz )
SET( ${proj}_MD5 76959f105cfbda3ba77889bc204f4bd2 )
# SET( ${proj}_REPOSITORY "${git_protocol}://github.com/RLovelett/eigen.git" )
# SET( ${proj}_GIT_TAG "2efca7e71fff7f17fcba0658f640b6ce8a53d469" ) # 3.2.7ish
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL ${${proj}_URL}
	URL_MD5 ${${proj}_MD5}
	# GIT_REPOSITORY ${${proj}_REPOSITORY}
	# GIT_TAG ${${proj}_GIT_TAG}
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

	DEPENDS
		${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( EIGEN_DIR ${CMAKE_BINARY_DIR}/${proj}-install )
SET( EIGEN_INCLUDE_DIR ${CMAKE_BINARY_DIR}/${proj}-install/include/eigen3 )

mark_as_superbuild(
	VARS
		EIGEN_DIR:PATH
		EIGEN_INCLUDE_DIR:PATH
	LABELS
		"FIND_PACKAGE"
	)

ExternalProject_Message( ${proj} "EIGEN_DIR: ${EIGEN_DIR}" )
ExternalProject_Message( ${proj} "EIGEN_INCLUDE_DIR: ${EIGEN_INCLUDE_DIR}" )
### --- End binary information
