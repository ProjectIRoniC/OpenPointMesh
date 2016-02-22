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
	JPEG
	PNG
	Qt
	TIFF
	zlib
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

### --- Project specific additions here
SET( EXTERNAL_PROJECT_OPTIONAL_ARGS )

IF( NOT APPLE )
	LIST( APPEND EXTERNAL_PROJECT_OPTIONAL_ARGS
		-DVTK_REQUIRED_OBJCXX_FLAGS:STRING="" # Should not be needed, but is always causing problems on mac
											# This is to prevent the garbage collection errors from creeping back in
	)
ELSE()
	LIST( APPEND EXTERNAL_PROJECT_OPTIONAL_ARGS
		-DVTK_USE_CARBON:BOOL=OFF
		-DVTK_USE_COCOA:BOOL=ON # Default to Cocoa, VTK/CMakeLists.txt will enable Carbon and disable cocoa if needed
		-DVTK_USE_X:BOOL=OFF
	)
ENDIF()

# Setup parallel builds if possible
IF( CMAKE_GENERATOR MATCHES ".*Makefiles.*" )
	# Use $(MAKE) as build command to propagate parallel make option
	SET( CUSTOM_BUILD_COMMAND BUILD_COMMAND "$(MAKE)" )
	SET( make_command_definition -DMAKE_COMMAND=$(MAKE) )
ELSE()
	SET( make_command_definition -DMAKE_COMMAND=${CMAKE_MAKE_PROGRAM} )
ENDIF()

# Qt ARGS
SET( QT_ARGS
	# Qt ARGS
	-DQT_QMAKE_EXECUTABLE:FILEPATH=${QT_QMAKE_EXECUTABLE}
	-DVTK_USE_QVTK_QTOPENGL:BOOL=ON
)

IF( WIN32 )
	LIST( APPEND QT_ARGS
		-DVTK_Group_Qt:BOOL=OFF
		-DModule_vtkGUISupportQt:BOOL=ON
		-DModule_vtkGUISupportQtOpenGL:BOOL=ON
		-DModule_vtkGUISupportQtSQL:BOOL=ON
		-DModule_vtkRenderingQt:BOOL=ON
		-DModule_vtkViewsQt:BOOL=ON
	)
ELSE()
	LIST( APPEND QT_ARGS
		-DVTK_Group_Qt:BOOL=ON
	)
ENDIF()

SET( ${proj}_CMAKE_OPTIONS
	${EXTERNAL_PROJECT_OPTIONAL_ARGS}
	# CMake ARGS
	-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
	-DCMAKE_CXX_FLAGS:STRING=${EP_COMMON_CXX_FLAGS}
	-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
	-DCMAKE_C_FLAGS:STRING=${EP_COMMON_C_FLAGS}
	-DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
	-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}/${proj}-install
	-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
	#-DCMAKE_INCLUDE_DIRECTORIES_BEFORE:BOOL=OFF
	# VTK Build ARGS
	-DBUILD_DOCUMENTATION:BOOL=OFF
	-DBUILD_TESTING:BOOL=OFF
	-DBUILD_EXAMPLES:BOOL=OFF
	-DVTK_USE_PARALLEL:BOOL=ON
	-DVTK_LEGACY_REMOVE:BOOL=ON
	# Qt ARGS
	${QT_ARGS}
	# ZLIB ARGS
	-DVTK_USE_SYSTEM_ZLIB:BOOL=ON
	-DZLIB_INCLUDE_DIR:PATH=${ZLIB_INCLUDE_DIR}
	-DZLIB_LIBRARY_DIR:PATH=${ZLIB_LIBRARY_DIR}
	-DZLIB_LIBRARY:FILEPATH=${ZLIB_LIBRARY}
	# JPEG ARGS
	-DVTK_USE_SYSTEM_JPEG:BOOL=ON
	-DJPEG_INCLUDE_DIR:PATH=${JPEG_INCLUDE_DIR}
	-DJPEG_LIBRARY_DIR:PATH=${JPEG_LIBRARY_DIR}
	-DJPEG_LIBRARY:FILEPATH=${JPEG_LIBRARY}
	# PNG ARGS
	-DVTK_USE_SYSTEM_PNG:BOOL=ON
	-DPNG_PNG_INCLUDE_DIR:PATH=${PNG_INCLUDE_DIR}
	-DPNG_LIBRARY_DIR:PATH=${PNG_LIBRARY_DIR}
	-DPNG_LIBRARY:FILEPATH=${PNG_LIBRARY}
	# TIFF ARGS
	-DVTK_USE_SYSTEM_TIFF:BOOL=ON
	-DTIFF_INCLUDE_DIR:PATH=${TIFF_INCLUDE_DIR}
	-DTIFF_LIBRARY_DIR:PATH=${TIFF_LIBRARY_DIR}
	-DTIFF_LIBRARY:FILEPATH=${TIFF_LIBRARY}
	# MNG ARGS
	-DMNG_INCLUDE_DIR:PATH=${MNG_INCLUDE_DIR}
	-DMNG_LIBRARY_DIR:PATH=${MNG_LIBRARY_DIR}
	-DMNG_LIBRARY:FILEPATH=${MNG_LIBRARY}
)

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/Kitware/VTK/archive/v6.3.0.tar.gz )
SET( ${proj}_MD5 b2f6fcc29fb42231bc2b025eccb41122 )
# SET( ${proj}_GIT_REPOSITORY "${git_protocol}://github.com/Kitware/VTK.git" )
# SET( ${proj}_GIT_TAG "v6.3.0" )
### --- End Project specific additions

ExternalProject_Add( ${proj}
	${${proj}_EP_ARGS}
	SOURCE_DIR ${SOURCE_DOWNLOAD_CACHE}/${proj}
	BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-build
	URL ${${proj}_URL}
	URL_MD5 ${${proj}_MD5}
	# GIT_REPOSITORY ${${proj}_REPOSITORY}
	# GIT_TAG ${${proj}_GIT_TAG}
	${CUSTOM_BUILD_COMMAND}
	CMAKE_ARGS -Wno-dev --no-warn-unused-cli
	CMAKE_CACHE_ARGS ${${proj}_CMAKE_OPTIONS}
	INSTALL_COMMAND ""
	DEPENDS ${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( VTK_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install )
SET( VTK_INCLUDE_DIR ${CMAKE_BINARY_DIR}/${proj}-install/include )
SET( VTK_LIBRARY_DIR ${CMAKE_BINARY_DIR}/${proj}-install/lib )

mark_as_superbuild(
	VARS
		VTK_DIR:PATH
		VTK_INCLUDE_DIR:PATH
		VTK_LIBRARY_DIR:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "VTK_DIR: ${VTK_DIR}" )
ExternalProject_Message( ${proj} "VTK_INCLUDE_DIR: ${VTK_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "VTK_LIBRARY_DIR: ${VTK_LIBRARY_DIR}" )
### --- End binary information
  