# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
SET( extProjName VTK ) # The find_package known name
SET( proj        VTK ) # The local name
SET( ${extProjName}_REQUIRED_VERSION "6.3" )  #If a required version is necessary, then set this, else leave blank
SET( VTK_VERSION_MAJOR 6 )

# Sanity checks
IF( DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR} )
	MESSAGE( FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})" )
ENDIF()

# Set dependency list
SET( ${proj}_DEPENDENCIES
	Qt
	zlib
)

# Include dependent projects if any
ExternalProject_Include_Dependencies( ${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES )

### --- Project specific additions here
SET( EXTERNAL_PROJECT_OPTIONAL_ARGS )
SET( VTK_WRAP_TCL OFF )
SET( VTK_WRAP_PYTHON OFF )
SET( BUILD_SHARED_LIBS OFF )
SET( VTK_WRAP_PYTHON OFF )

IF( NOT APPLE )
	LIST( APPEND EXTERNAL_PROJECT_OPTIONAL_ARGS
		#-DDESIRED_QT_VERSION:STRING=4 # Unused
		-DVTK_USE_GUISUPPORT:BOOL=ON
		-DVTK_USE_QVTK_QTOPENGL:BOOL=${${PRIMARY_PROJECT_NAME}_USE_QT}
		-DVTK_Group_Qt:BOOL=${${PRIMARY_PROJECT_NAME}_USE_QT} ##VTK6
		-DQT_QMAKE_EXECUTABLE:FILEPATH=${QT_QMAKE_EXECUTABLE}
		-DVTK_REQUIRED_OBJCXX_FLAGS:STRING="" # Should not be needed, but is always causing problems on mac
											# This is to prevent the garbage collection errors from creeping back in
	)
ELSE()
	LIST( APPEND EXTERNAL_PROJECT_OPTIONAL_ARGS
		-DVTK_USE_CARBON:BOOL=OFF
		-DVTK_USE_COCOA:BOOL=ON # Default to Cocoa, VTK/CMakeLists.txt will enable Carbon and disable cocoa if needed
		-DVTK_USE_X:BOOL=OFF
		-DVTK_USE_GUISUPPORT:BOOL=ON
		-DVTK_USE_QVTK_QTOPENGL:BOOL=${${PRIMARY_PROJECT_NAME}_USE_QT}
		-DVTK_Group_Qt:BOOL=${${PRIMARY_PROJECT_NAME}_USE_QT}  ## VTK6
		-DQT_QMAKE_EXECUTABLE:FILEPATH=${QT_QMAKE_EXECUTABLE}
	)
ENDIF()

LIST( APPEND EXTERNAL_PROJECT_OPTIONAL_ARGS
	-DModule_vtkGUISupportQt:BOOL=${${PRIMARY_PROJECT_NAME}_USE_QT}
	-DModule_vtkGUISupportQtOpenGL:BOOL=${${PRIMARY_PROJECT_NAME}_USE_QT}
)

IF( VTK_WRAP_TCL )
	LIST( APPEND EXTERNAL_PROJECT_OPTIONAL_ARGS
		-DTCL_INCLUDE_PATH:PATH=${TCL_INCLUDE_PATH}
		-DTK_INCLUDE_PATH:PATH=${TK_INCLUDE_PATH}
		-DTCL_LIBRARY:FILEPATH=${TCL_LIBRARY}
		-DTK_LIBRARY:FILEPATH=${TK_LIBRARY}
		-DTCL_TCLSH:FILEPATH=${TCL_TCLSH}
	)
ENDIF()

IF( CMAKE_GENERATOR MATCHES ".*Makefiles.*" )
	# Use $(MAKE) as build command to propagate parallel make option
	SET( CUSTOM_BUILD_COMMAND BUILD_COMMAND "$(MAKE)" )
	SET( make_command_definition -DMAKE_COMMAND=$(MAKE) )
ELSE()
	SET( make_command_definition -DMAKE_COMMAND=${CMAKE_MAKE_PROGRAM} )
ENDIF()

IF( UNIX )
	CONFIGURE_FILE( SuperBuild/VTK_build_step.cmake.in
		${CMAKE_CURRENT_BINARY_DIR}/VTK_build_step.cmake
		@ONLY
	)
	
	SET( CUSTOM_BUILD_COMMAND BUILD_COMMAND ${CMAKE_COMMAND}
		${make_command_definition}
		-P ${CMAKE_CURRENT_BINARY_DIR}/VTK_build_step.cmake
	)
ENDIF()

# Download tar source when possible to speed up build time
SET( ${proj}_URL https://github.com/Kitware/VTK/archive/v6.3.0.tar.gz )
SET( ${proj}_MD5 b2f6fcc29fb42231bc2b025eccb41122 )
# SET( ${proj}_GIT_REPOSITORY "${git_protocol}://github.com/Kitware/VTK.git" )
# SET( ${proj}_GIT_TAG "v6.3.0" )  # 6.3.0
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
	CMAKE_CACHE_ARGS
		${EXTERNAL_PROJECT_OPTIONAL_ARGS}
		-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
		-DCMAKE_CXX_FLAGS:STRING=${ep_common_cxx_flags}
		-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
		-DCMAKE_C_FLAGS:STRING=${ep_common_c_flags}
		-DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
		-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}/${proj}-install
		-DCMAKE_INCLUDE_DIRECTORIES_BEFORE:BOOL=OFF
		-DBUILD_TESTING:BOOL=OFF
		-DBUILD_EXAMPLES:BOOL=OFF
		-DBUILD_SHARED_LIBS:BOOL=OFF
		-DVTK_USE_PARALLEL:BOOL=ON
		-DVTK_DEBUG_LEAKS:BOOL=${VTK_DEBUG_LEAKS}
		-DVTK_LEGACY_REMOVE:BOOL=ON
		-DVTK_WRAP_TCL:BOOL=${VTK_WRAP_TCL}
		-DModule_vtkIOXML:BOOL=ON
		-DModule_vtkIOXMLParser:BOOL=ON
		${VTK_QT_ARGS}
		${VTK_MAC_ARGS}
		# ZLIB ARGS
		-D${proj}_USE_SYSTEM_ZLIB:BOOL=OFF
		-DZLIB_ROOT:PATH=${zlib_DIR}
		-DZLIB_INCLUDE_DIR:PATH=${zlib_INCLUDE_DIR}
		-DZLIB_LIBRARY:FILEPATH=${zlib_LIBRARY}
	INSTALL_COMMAND ""
	DEPENDS
		${${proj}_DEPENDENCIES}
)

### --- Set binary information
SET( ${extProjName}_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-build )
SET( ${extProjName}_INCLUDE_DIR ${CMAKE_BINARY_DIR}/${proj}-install/include/VTK )
SET( VTK_SOURCE_DIR ${CMAKE_BINARY_DIR}/${proj} )
SET( PNG_INCLUDE_DIR ${VTK_SOURCE_DIR}/Utilities/vtkpng )
SET( PNG_LIBRARY_DIR ${VTK_DIR}/bin )

IF( CMAKE_CONFIGURATION_TYPES )
	SET( PNG_LIBRARY_DIR ${PNG_LIBRARY_DIR}/${CMAKE_CFG_INTDIR} )
ENDIF()

IF( WIN32 )
	SET( PNG_LIBRARY ${PNG_LIBRARY_DIR}/vtkpng.lib )
ELSEIF(APPLE)
	SET( PNG_LIBRARY ${PNG_LIBRARY_DIR}/libvtkpng.dylib )
ELSE()
	SET( PNG_LIBRARY ${PNG_LIBRARY_DIR}/libvtkpng.so )
ENDIF()

mark_as_superbuild(
	VARS
		VTK_DIR:PATH
		VTK_INCLUDE_DIR:PATH
		VTK_VERSION_MAJOR:STRING
		VTK_SOURCE_DIR:PATH
		PNG_INCLUDE_DIR:PATH
		PNG_LIBRARY_DIR:PATH
		PNG_LIBRARY:PATH
	LABELS
		"FIND_PACKAGE"
)

ExternalProject_Message( ${proj} "VTK_DIR: ${VTK_DIR}" )
ExternalProject_Message( ${proj} "VTK_INCLUDE_DIR: ${VTK_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "VTK_SOURCE_DIR: ${VTK_SOURCE_DIR}" )
ExternalProject_Message( ${proj} "PNG_INCLUDE_DIR: ${PNG_INCLUDE_DIR}" )
ExternalProject_Message( ${proj} "PNG_LIBRARY_DIR: ${PNG_LIBRARY_DIR}" )
ExternalProject_Message( ${proj} "PNG_LIBRARY: ${PNG_LIBRARY}" )
### --- End binary information
  