# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName PCL ) # The find_package known name
SET( proj        PCL ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "1.7" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	Boost
	Eigen
	FLANN
	#OpenNI2
	qhull	
	Qt
	VTK
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-build )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

SET( OPENNI2_INCLUDE_DIR "C:/OpenNI2/Include" )
SET( OPENNI2_LIBRARY "C:/OpenNI2/Lib/OpenNI2.lib" )

### --- Project specific additions here
SET( ${proj}_CMAKE_OPTIONS
	# CMake Build ARGS
	-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
	-DCMAKE_CXX_FLAGS:STRING=${EP_COMMON_CXX_FLAGS}
	-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
	-DCMAKE_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
	-DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
	-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
	-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
	# PCL Build ARGS
	-DPCL_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
	#-DBUILD_TESTING:BOOL=OFF
	#-DBUILD_TESTS:BOOL=OFF
	-DWITH_DOCS:BOOL=OFF
	-DWITH_OPENNI:BOOL=OFF
	-DBUILD_OPENNI:BOOL=OFF
	-DWITH_OPENNI2:BOOL=OFF
	-DBUILD_OPENNI2:BOOL=OFF
	-DBUILD_all_in_one_installer:BOOL=OFF
	-DBUILD_apps:BOOL=ON
	-DBUILD_apps_cloud_composer:BOOL=ON
	-DBUILD_apps_modeler:BOOL=ON
	-DBUILD_apps_point_cloud_editor:BOOL=ON
	-DBUILD_common:BOOL=ON
	-DBUILD_examples:BOOL=OFF
	-DBUILD_features:BOOL=ON
	-DBUILD_filters:BOOL=ON
	-DBUILD_geometry:BOOL=ON
	-DBUILD_global_tests:BOOL=OFF
	-DBUILD_io:BOOL=ON
	-DBUILD_kdtree:BOOL=ON
	-DBUILD_keypoints:BOOL=ON
	-DBUILD_octree:BOOL=ON
	-DBUILD_outofcore:BOOL=ON
	-DBUILD_people:BOOL=ON
	-DBUILD_recognition:BOOL=ON
	-DBUILD_registration:BOOL=ON
	-DBUILD_sample_consensus:BOOL=ON
	-DBUILD_search:BOOL=ON
	-DBUILD_segmentation:BOOL=ON
	-DBUILD_surface:BOOL=ON
	-DBUILD_surface_on_nurbs:BOOL=ON
	-DBUILD_tools:BOOL=ON
	-DBUILD_tracking:BOOL=ON
	-DBUILD_visualization:BOOL=ON
	# Boost ARGS
	-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
	-DBOOST_ROOT:PATH=${BOOST_DIR}
	-DBoost_INCLUDE_DIR:PATH=${BOOST_INCLUDE_DIR}
	-DBoost_LIBRARY_DIRS:PATH=${BOOST_LIBRARY_DIR}
	-DBoost_LIBRARY_DIR_DEBUG:PATH=${BOOST_LIBRARY_DIR}
	-DBoost_LIBRARY_DIR_RELEASE:PATH=${BOOST_LIBRARY_DIR}
	# EIGEN ARGS
	-DEIGEN_INCLUDE_DIR:PATH=${EIGEN_INCLUDE_DIR}
	# FLANN ARGS
	-DFLANN_INCLUDE_DIR:PATH=${FLANN_INCLUDE_DIR}
	-DFLANN_LIBRARY:FILEPATH=${FLANN_LIBRARY}
	# JPEG ARGS
	-DJPEG_INCLUDE_DIR:PATH=${JPEG_INCLUDE_DIR}
	-DJPEG_LIBRARY:FILEPATH=${JPEG_LIBRARY}
	# MNG ARGS
	-DMNG_INCLUDE_DIR:PATH=${MNG_INCLUDE_DIR}
	-DMNG_LIBRARY:FILEPATH=${MNG_LIBRARY}
	# OPENNI2 ARGS
	-DOPENNI2_INCLUDE_DIRS:PATH=${OPENNI2_INCLUDE_DIR}
	-DOPENNI2_LIBRARY:FILEPATH=${OPENNI2_LIBRARY}
	# PNG ARGS
	-DPNG_PNG_INCLUDE_DIR:PATH=${PNG_INCLUDE_DIR}
	-DPNG_LIBRARY:FILEPATH=${PNG_LIBRARY}
	# QHULL ARGS
	-DQHULL_INCLUDE_DIR:PATH=${QHULL_INCLUDE_DIR}
	-DQHULL_LIBRARY:FILEPATH=${QHULL_LIBRARY}
	# QT ARGS
	-DQT_QMAKE_EXECUTABLE:FILEPATH=${QT_QMAKE_EXECUTABLE}
	# TIFF ARGS
	-DTIFF_INCLUDE_DIR:PATH=${TIFF_INCLUDE_DIR}
	-DTIFF_LIBRARY:FILEPATH=${TIFF_LIBRARY}
	# VTK ARGS
	-DVTK_DIR:PATH=${VTK_BUILD_DIR}
	# ZLIB ARGS
	-DZLIB_INCLUDE_DIR:PATH=${ZLIB_INCLUDE_DIR}
	-DZLIB_LIBRARY:FILEPATH=${ZLIB_LIBRARY}
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/PointCloudLibrary/pcl/archive/pcl-1.7.2.tar.gz )
SET( ${proj}_MD5 02c72eb6760fcb1f2e359ad8871b9968 )
# SET( ${proj}_REPOSITORY "${git_protocol}://github.com/PointCloudLibrary/pcl.git" )
# SET( ${proj}_GIT_TAG "pcl-1.7.2" )  # 1.7.2
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL		${${proj}_URL}
	URL_MD5	${${proj}_MD5}
	# GIT_REPOSITORY	${${proj}_REPOSITORY}
	# GIT_TAG 			${${proj}_GIT_TAG}
	SOURCE_DIR	${${proj}_SOURCE_DIR}
	BINARY_DIR	${${proj}_BUILD_DIR}
	INSTALL_DIR	${${proj}_INSTALL_DIR}
	LOG_CONFIGURE	0  # Wrap configure in script to ignore log output from dashboards
	LOG_BUILD		0  # Wrap build in script to to ignore log output from dashboards
	LOG_TEST		0  # Wrap test in script to to ignore log output from dashboards
	LOG_INSTALL		0  # Wrap install in script to to ignore log output from dashboards
	${cmakeversion_external_update} "${cmakeversion_external_update_value}"
	CMAKE_GENERATOR		${gen}
	CMAKE_ARGS			-Wno-dev --no-warn-unused-cli
	CMAKE_CACHE_ARGS	${${proj}_CMAKE_OPTIONS}
	DEPENDS	${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( PCL_DIR ${${proj}_INSTALL_DIR} )
SET( PCL_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( PCL_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( PCL_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )

mark_as_superbuild(
	VARS
		PCL_DIR:PATH
		PCL_BUILD_DIR:PATH
		PCL_INCLUDE_DIR:PATH
		PCL_LIBRARY_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "PCL_DIR: ${PCL_DIR}" )
ExternalProject_Message( ${proj} "PCL_BUILD_DIR: ${PCL_BUILD_DIR}" )
ExternalProject_Message( ${proj} "PCL_INCLUDE_DIR: ${PCL_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "PCL_LIBRARY_DIR: ${PCL_LIBRARY_DIR}" )
### --- End binary information
