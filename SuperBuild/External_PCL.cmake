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
	OpenNI2
	Qhull	
	Qt
	VTK
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-build )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( PCL_CMAKE_PREFIX_PATH
	${BOOST_DIR}
	${BZIP2_DIR}
	${EIGEN_DIR}
	${EXPAT_DIR}
	${FLANN_DIR}
	${FONTCONFIG_DIR}
	${FREETYPE_DIR}
	${JPEG_DIR}
	${LCMS_DIR}
	${LIBUSB_1_DIR}
	${MNG_DIR}
	${OPENNI2_DIR}
	${PNG_DIR}
	${QHULL_DIR}
	${TIFF_DIR}
	${VTK_DIR}
	${ZLIB_DIR}
)

SET( ${proj}_CMAKE_OPTIONS
	# CMake Build ARGS
	-DCMAKE_INCLUDE_DIRECTORIES_BEFORE:BOOL=${CMAKE_INCLUDE_DIRECTORIES_BEFORE}
	-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
	-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
	-DCMAKE_SKIP_BUILD_RPATH:BOOL=${CMAKE_SKIP_BUILD_RPATH}
	-DCMAKE_BUILD_WITH_INSTALL_RPATH:BOOL=${CMAKE_BUILD_WITH_INSTALL_RPATH}
	-DCMAKE_INSTALL_RPATH:PATH=${CMAKE_INSTALL_RPATH}
	-DCMAKE_INSTALL_RPATH_USE_LINK_PATH:BOOL=${CMAKE_INSTALL_RPATH_USE_LINK_PATH}
	-DCMAKE_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
	-DCMAKE_CXX_FLAGS:STRING=${EP_COMMON_CXX_FLAGS}
	-DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
	-DCMAKE_PREFIX_PATH:PATH=${PCL_CMAKE_PREFIX_PATH}
	-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
	# PCL Build ARGS
	-DPCL_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
	-DWITH_DOCS:BOOL=OFF
	-DWITH_OPENNI:BOOL=OFF
	-DWITH_OPENNI2:BOOL=ON
	-DBUILD_all_in_one_installer:BOOL=OFF
	-DBUILD_apps:BOOL=OFF
	#-DBUILD_apps_cloud_composer:BOOL=OFF
	#-DBUILD_apps_modeler:BOOL=OFF
	#-DBUILD_apps_point_cloud_editor:BOOL=OFF
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
	-DBUILD_surface_on_nurbs:BOOL=OFF
	-DBUILD_tools:BOOL=OFF
	-DBUILD_tracking:BOOL=ON
	-DBUILD_visualization:BOOL=ON
	# BOOST ARGS
	-DBOOST_ROOT:PATH=${BOOST_DIR}
	-DBOOST_INCLUDEDIR:PATH=${BOOST_INCLUDE_DIR}
	-DBOOST_LIBRARYDIR:PATH=${BOOST_LIBRARY_DIR}
	# OpenGL ARGS
	-DOPENGL_gl_LIBRARY:FILEPATH=${OPENGL_gl_LIBRARY}
	# GLUT ARGS
	#-DGLUT_glut_LIBRARY:FILEPATH=${GLUT_glut_LIBRARY}
	#-DGLUT_Xi_LIBRARY:FILEPATH=${GLUT_Xi_LIBRARY}
	#-DGLUT_Xmu_LIBRARY:FILEPATH=${GLUT_Xmu_LIBRARY}
	# QT ARGS
	-DQT_QMAKE_EXECUTABLE:FILEPATH=${QT_QMAKE_EXECUTABLE}
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/PointCloudLibrary/pcl/archive/pcl-1.7.2.tar.gz )
SET( ${proj}_MD5 02c72eb6760fcb1f2e359ad8871b9968 )
#SET( ${proj}_REPOSITORY "${git_protocol}://github.com/PointCloudLibrary/pcl.git" )
#SET( ${proj}_GIT_TAG "master" )
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	URL					${${proj}_URL}
	URL_MD5				${${proj}_MD5}
	# GIT_REPOSITORY	${${proj}_REPOSITORY}
	# GIT_TAG 			${${proj}_GIT_TAG}
	SOURCE_DIR			${${proj}_SOURCE_DIR}
	BINARY_DIR			${${proj}_BUILD_DIR}
	INSTALL_DIR			${${proj}_INSTALL_DIR}
	LOG_DOWNLOAD		${EP_LOG_DOWNLOAD}
	LOG_UPDATE			${EP_LOG_UPDATE}
	LOG_CONFIGURE		${EP_LOG_CONFIGURE}
	LOG_BUILD			${EP_LOG_BUILD}
	LOG_TEST			${EP_LOG_TEST}
	LOG_INSTALL			${EP_LOG_INSTALL}
	CMAKE_GENERATOR		${gen}
	CMAKE_ARGS			${EP_CMAKE_ARGS}
	CMAKE_CACHE_ARGS	${${proj}_CMAKE_OPTIONS}
	DEPENDS				${${proj}_DEPENDENCIES}
)

# Our project has better find package modules than PCL
# so we are going to remove/rename the files so CMake uses our versions
ExternalProject_Add_Step( ${proj} remove_PCL_FindEigen.cmake
	COMMAND ${CMAKE_COMMAND}
			-E rename ${${proj}_SOURCE_DIR}/cmake/Modules/FindEigen.cmake ${${proj}_SOURCE_DIR}/cmake/Modules/RENAMED_BY_SUPERBUILD_FindEigen.cmake

	DEPENDEES download
	DEPENDERS configure
)

ExternalProject_Add_Step( ${proj} remove_PCL_FindFLANN.cmake
	COMMAND ${CMAKE_COMMAND}
			-E rename ${${proj}_SOURCE_DIR}/cmake/Modules/FindFLANN.cmake ${${proj}_SOURCE_DIR}/cmake/Modules/RENAMED_BY_SUPERBUILD_FindFLANN.cmake

	DEPENDEES download
	DEPENDERS configure
)

ExternalProject_Add_Step( ${proj} remove_PCL_Findlibusb-1.0.cmake
	COMMAND ${CMAKE_COMMAND}
			-E rename ${${proj}_SOURCE_DIR}/cmake/Modules/Findlibusb-1.0.cmake ${${proj}_SOURCE_DIR}/cmake/Modules/RENAMED_BY_SUPERBUILD_Findlibusb-1.0.cmake

	DEPENDEES download
	DEPENDERS configure
)

ExternalProject_Add_Step( ${proj} remove_PCL_FindOpenNI2.cmake
	COMMAND ${CMAKE_COMMAND}
			-E rename ${${proj}_SOURCE_DIR}/cmake/Modules/FindOpenNI2.cmake ${${proj}_SOURCE_DIR}/cmake/Modules/RENAMED_BY_SUPERBUILD_FindOpenNI2.cmake

	DEPENDEES download
	DEPENDERS configure
)

ExternalProject_Add_Step( ${proj} remove_PCL_FindQhull.cmake
	COMMAND ${CMAKE_COMMAND}
			-E rename ${${proj}_SOURCE_DIR}/cmake/Modules/FindQhull.cmake ${${proj}_SOURCE_DIR}/cmake/Modules/RENAMED_BY_SUPERBUILD_FindQhull.cmake

	DEPENDEES download
	DEPENDERS configure
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
