# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName VTK ) # The find_package known name
SET( proj        VTK ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "6.3" )  #If a required version is necessary, then set this, else leave blank

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	BZip2
	Eigen
	EXPAT
	FontConfig
	Freetype
	JPEG
	LCMS
	MNG
	PNG
	Qt
	TIFF
	ZLIB
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

# Set directories
SET( ${proj}_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-build )
SET( ${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( ${proj}_SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj} )

### --- Project specific additions here
SET( VTK_CMAKE_PREFIX_PATH
	${BZIP2_DIR}
	${EIGEN_DIR}
	${EXPAT_DIR}
	${FONTCONFIG_DIR}
	${FREETYPE_DIR}
	${JPEG_DIR}
	${LCMS_DIR}
	${MNG_DIR}
	${PNG_DIR}
	${TIFF_DIR}
	${ZLIB_DIR}
)

# VTK ARGS
SET( VTK_ARGS )
IF( NOT APPLE )
	LIST( APPEND VTK_ARGS
		-DVTK_REQUIRED_OBJCXX_FLAGS:STRING="" # Should not be needed, but is always causing problems on mac
											# This is to prevent the garbage collection errors from creeping back in
	)
ELSE()
	LIST( APPEND VTK_ARGS
		-DVTK_USE_CARBON:BOOL=OFF
		-DVTK_USE_COCOA:BOOL=ON # Default to Cocoa, VTK/CMakeLists.txt will enable Carbon and disable cocoa if needed
		-DVTK_USE_X:BOOL=OFF
	)
ENDIF()

# Qt ARGS
SET( QT_ARGS
	# Qt ARGS
	-DQT_QMAKE_EXECUTABLE:FILEPATH=${QT_QMAKE_EXECUTABLE}
	-DVTK_USE_QVTK_QTOPENGL:BOOL=ON
	-DVTK_Group_Qt:BOOL=OFF
	-DModule_vtkGUISupportQt:BOOL=ON
	-DModule_vtkGUISupportQtOpenGL:BOOL=ON
)

SET( ${proj}_CMAKE_OPTIONS
	${VTK_ARGS}
	# CMake build ARGS
	-DCMAKE_INCLUDE_DIRECTORIES_BEFORE:BOOL=${CMAKE_INCLUDE_DIRECTORIES_BEFORE}
	-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=${CMAKE_POSITION_INDEPENDENT_CODE}
	-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
	-DCMAKE_SKIP_BUILD_RPATH:BOOL=${CMAKE_SKIP_BUILD_RPATH}
	-DCMAKE_BUILD_WITH_INSTALL_RPATH:BOOL=${CMAKE_BUILD_WITH_INSTALL_RPATH}
	-DCMAKE_INSTALL_RPATH:PATH=${CMAKE_INSTALL_RPATH}
	-DCMAKE_INSTALL_RPATH_USE_LINK_PATH:BOOL=${CMAKE_INSTALL_RPATH_USE_LINK_PATH}
	-DCMAKE_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
	-DCMAKE_CXX_FLAGS:STRING=${EP_COMMON_CXX_FLAGS}
	-DCMAKE_PREFIX_PATH:PATH=${VTK_CMAKE_PREFIX_PATH}
	-DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
	# VTK Build ARGS
	-DBUILD_DOCUMENTATION:BOOL=OFF
	-DBUILD_TESTING:BOOL=OFF
	-DBUILD_EXAMPLES:BOOL=OFF
	-DVTK_USE_PARALLEL:BOOL=ON
	-DVTK_LEGACY_REMOVE:BOOL=ON
	# FreeType ARGS
	-DVTK_USE_SYSTEM_FREETYPE:BOOL=ON
	-DModule_vtkRenderingFreeTypeFontConfig:BOOL=ON
	# JPEG ARGS
	-DVTK_USE_SYSTEM_JPEG:BOOL=ON
	# PNG ARGS
	-DVTK_USE_SYSTEM_PNG:BOOL=ON
	# Qt ARGS
	${QT_ARGS}
	# TIFF ARGS
	-DVTK_USE_SYSTEM_TIFF:BOOL=ON
	# ZLIB ARGS
	-DVTK_USE_SYSTEM_ZLIB:BOOL=ON
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/Kitware/VTK/archive/v6.3.0.tar.gz )
SET( ${proj}_MD5 b2f6fcc29fb42231bc2b025eccb41122 )
# SET( ${proj}_GIT_REPOSITORY "${git_protocol}://github.com/Kitware/VTK.git" )
# SET( ${proj}_GIT_TAG "v6.3.0" )
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

# VTK's version of FindFontConfig.cmake does not link the EXPAT dependency
# so we are going to remove/rename the file so CMake uses our version
ExternalProject_Add_Step( ${proj} remove_VTK_FindFontConfg.cmake
	COMMAND ${CMAKE_COMMAND}
			-E rename ${${proj}_SOURCE_DIR}/CMake/FindFontConfig.cmake ${${proj}_SOURCE_DIR}/CMake/RENAMED_BY_SUPERBUILD_FindFontConfig.cmake

	DEPENDEES download
	DEPENDERS configure
)

### --- Set binary information
SET( VTK_DIR ${${proj}_INSTALL_DIR} )
SET( VTK_BUILD_DIR ${${proj}_BUILD_DIR} )
SET( VTK_INCLUDE_DIR ${${proj}_INSTALL_DIR}/include )
SET( VTK_LIBRARY_DIR ${${proj}_INSTALL_DIR}/lib )

mark_as_superbuild(
	VARS
		VTK_DIR:PATH
		VTK_BUILD_DIR:PATH
		VTK_INCLUDE_DIR:PATH
		VTK_LIBRARY_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "VTK_DIR: ${VTK_DIR}" )
ExternalProject_Message( ${proj} "VTK_BUILD_DIR: ${VTK_BUILD_DIR}" )
ExternalProject_Message( ${proj} "VTK_INCLUDE_DIR: ${VTK_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "VTK_LIBRARY_DIR: ${VTK_LIBRARY_DIR}" )
### --- End binary information
