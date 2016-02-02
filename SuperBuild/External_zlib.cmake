# Make sure that the ExtProjName/IntProjName variables are unique globally
# even if other External_${ExtProjName}.cmake files are sourced by
# ExternalProject_Include_Dependencies
set(extProjName zlib) #The find_package known name
set(proj        zlib) #This local name
set(${extProjName}_REQUIRED_VERSION "")  #If a required version is necessary, then set this, else leave blank

# Sanity checks
if(DEFINED ${extProjName}_DIR AND NOT EXISTS ${${extProjName}_DIR})
  message(FATAL_ERROR "${extProjName}_DIR variable is defined but corresponds to non-existing directory (${${extProjName}_DIR})")
endif()

# Set dependency list
set(${proj}_DEPENDENCIES "")

# Include dependent projects if any
ExternalProject_Include_Dependencies(${proj} PROJECT_VAR proj DEPENDS_VAR ${proj}_DEPENDENCIES)

#if(${CMAKE_PROJECT_NAME}_USE_SYSTEM_${proj})
#  unset(zlib_DIR CACHE)
#  find_package(ZLIB REQUIRED)
#  set(ZLIB_INCLUDE_DIR ${ZLIB_INCLUDE_DIRS})
#  set(ZLIB_LIBRARY ${ZLIB_LIBRARIES})
#endif()

if(NOT ( DEFINED "USE_SYSTEM_${extProjName}" AND "${USE_SYSTEM_${extProjName}}" ) )
  ### --- Project specific additions here
  set(${proj}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${proj}-install)
  set(${proj}_CMAKE_OPTIONS
	  #C++11 shouldn't be needed
      #-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
      #-DCMAKE_CXX_FLAGS:STRING=${ep_common_cxx_flags}
      -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
      -DCMAKE_C_FLAGS:STRING=${ep_common_c_flags}
      -DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
      -DCMAKE_INSTALL_PREFIX:PATH=${${proj}_INSTALL_DIR}
    )
  ### --- End Project specific additions

  set(${proj}_REPOSITORY "${git_protocol}://github.com/commontk/zlib.git")
  set(${proj}_GIT_TAG "66a753054b356da85e1838a081aa94287226823e")
  ExternalProject_Add(${proj}
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
    CMAKE_CACHE_ARGS
	  ${${proj}_CMAKE_OPTIONS}
    DEPENDS
      ${${proj}_DEPENDENCIES}
    )
	
  ### --- Set binary information here
  set(${extProjName}_DIR ${CMAKE_BINARY_DIR}/${proj}-install)
  set(${extProjName}_ROOT ${zlib_DIR})
  set(${extProjName}_INCLUDE_DIR ${zlib_DIR}/include)
  if(WIN32)
    set(${extProjName}_LIBRARY ${zlib_DIR}/lib/zlib.lib)
  else()
    set(${extProjName}_LIBRARY ${zlib_DIR}/lib/libzlib.a)
  endif()
  ### --- End binary information here
else()
  # The project is provided using zlib_DIR, nevertheless since other project may depend on zlib,
  # let's add an 'empty' one
  ExternalProject_Add_Empty(${proj} "${${proj}_DEPENDENCIES}")
endif()

mark_as_superbuild(
  VARS
    ${extProjName}_INCLUDE_DIR:PATH
    ${extProjName}_LIBRARY:FILEPATH
    ${extProjName}_ROOT:PATH
  LABELS
	"FIND_PACKAGE"
)

ExternalProject_Message(${extProjName} "${extProjName}_INCLUDE_DIR:${${extProjName}_INCLUDE_DIR}")
ExternalProject_Message(${extProjName} "${extProjName}_LIBRARY:${${extProjName}_LIBRARY}")
if(ZLIB_ROOT)
  ExternalProject_Message(${extProjName} "${extProjName}_ROOT:${${extProjName}_ROOT}")
endif()
